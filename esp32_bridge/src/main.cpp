#include <Arduino.h>
#include <ESPmDNS.h>
#include "Mcp23017Bus.h"
#include "Motor.h"
#include "RobotController.h"
#include "PanTiltController.h"
#include "WebDashboard.h"
#include "DebugLog.h"

// ============================================================================
// CONFIGURATION - Modify these values as needed
// ============================================================================

// WiFi credentials
const char* WIFI_SSID = "WiFi-";
const char* WIFI_PASSWORD = "";

// mDNS hostname (access via http://robot.local)
const char* MDNS_HOSTNAME = "robot";

// Robot physical parameters
const RobotParams ROBOT_PARAMS = {
    0.144f,     // wheelDiameterM: Wheel diameter in meters (90mm)
    1425.0f,     // encoderCPR: Encoder counts per revolution
    0.20f,      // wheelBaseM: Distance between left/right wheels (200mm)
    1.0f        // maxSpeedMPS: Maximum speed in m/s
};

// ============================================================================
// HARDWARE PINS
// ============================================================================

// Shared pins
constexpr int PIN_SDA = 16;
constexpr int PIN_SCL = 15;

// MCP23017 address
constexpr uint8_t MCP_ADDR = 0x20;

// MCP23017 LED pins
constexpr uint8_t MCP_RED_LED   = 1;  // GPA1
constexpr uint8_t MCP_GREEN_LED = 2;  // GPA2

// Auxiliary output
constexpr int PIN_AUX_OUT = 42;

// Motor pin configurations
// MotorPins: mcpINA, mcpINB, mcpSEL0, mcpSEL1, pwmGPIO, csAdcGPIO, encA, encB
// SEL0=GPA5(5), SEL1=GPA6(6) are shared across all four drivers
const MotorPins M1_PINS = {3,  0,  5, 6, 8,  4, 17, 7};
const MotorPins M2_PINS = {15, 12, 5, 6, 48, 1, 38, 39};  // encA/B swapped (left side)
const MotorPins M3_PINS = {11, 8,  5, 6, 13, 2, 14, 47};  // encA/B swapped (left side)
const MotorPins M4_PINS = {7,  4,  5, 6, 9,  5, 11, 12};

// Motor configuration
const MotorConfig MOTOR_CONFIG = {20000, 10, false, 0.14f, 0.0f, 3.0f};

// ============================================================================
// FREERTOS CONFIGURATION
// ============================================================================

constexpr uint32_t MOTOR_TASK_STACK_SIZE = 4096;
constexpr uint32_t MOTOR_TASK_PRIORITY = 5;
constexpr uint32_t MOTOR_UPDATE_INTERVAL_MS = 10;

constexpr uint32_t TELEMETRY_TASK_STACK_SIZE = 4096;
constexpr uint32_t TELEMETRY_TASK_PRIORITY = 2;
constexpr uint32_t TELEMETRY_UPDATE_INTERVAL_MS = 50;

constexpr uint32_t SERIAL_CMD_TASK_STACK_SIZE = 4096;
constexpr uint32_t SERIAL_CMD_TASK_PRIORITY   = 3;
constexpr uint32_t SERIAL_WATCHDOG_MS         = 1000;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Mcp23017Bus mcp;
Motor* motors[4];
RobotController*   robot    = nullptr;
PanTiltController* panTilt  = nullptr;
WebDashboard*      dashboard = nullptr;

// Task handles
TaskHandle_t motorTaskHandle     = nullptr;
TaskHandle_t telemetryTaskHandle = nullptr;
TaskHandle_t serialCmdTaskHandle = nullptr;

// Serial command watchdog — set to millis() on each motor command, 0 = no command received yet
volatile uint32_t lastMotorCommandMs = 0;

// Auxiliary pin state — shared with WebDashboard for telemetry
volatile bool g_auxPinHigh = false;

// Emergency stop latch — set by AUX,estop or /api/estop, cleared by
// AUX,estopclear or /api/estop?state=0. While active, incoming CMD
// velocity commands are ignored.
volatile bool g_eStopActive = false;

// Set by the serial watchdog on comms timeout, cleared by the next CMD.
//
// Both flags are only *requests*: motorControlTask is the single writer
// of motor state and performs the disable+brake (and the re-enable when
// the condition clears). Braking directly from another task/core could
// be overwritten by an in-flight PID cycle's setDuty() calls.
volatile bool g_watchdogBrake = false;


// Telemetry data (shared between tasks)
struct TelemetryData {
    int64_t encoderCounts[4];
    float motorCurrents[4];
    float actualSpeed;
    SemaphoreHandle_t mutex;
} telemetry;

// ============================================================================
// SERIAL COMMAND PROTOCOL (ROS2 pan/tilt diff drive interface)
//
// Messages FROM PC → ESP32 (terminated by \n):
//   "GET\n"
//       — request current state; ESP32 responds immediately with STATE message
//   "CMD,lf_vel,lr_vel,rf_vel,rr_vel,pan_pos,tilt_pos\n"
//       — set per-wheel speeds (cm/s, float) and servo positions (radians)
//   "AUX,command_name,arg\n"
//       — auxiliary commands, including:
//           AUX,estop       — emergency stop: brake + disable motors, ignore CMD until cleared
//           AUX,estopclear  — clear emergency stop, resume normal CMD control
//
// Messages FROM ESP32 → PC (in response to GET):
//   "STATE,lf_pos,lr_pos,rf_pos,rr_pos,lf_vel,lr_vel,rf_vel,rr_vel,pan_pos,tilt_pos\n"
//       — encoder counts (int) and wheel velocities (cm/s, float), servo positions (radians)
//
// Wheel order: lf=FL(M3), lr=RL(M2), rf=FR(M4), rr=RR(M1)
// ============================================================================

void sendStateMessage() {
    // Encoder positions (cumulative counts, integer)
    int64_t lf_pos = robot->getEncoderCount(FRONT_LEFT);
    int64_t lr_pos = robot->getEncoderCount(REAR_LEFT);
    int64_t rf_pos = robot->getEncoderCount(FRONT_RIGHT);
    int64_t rr_pos = robot->getEncoderCount(REAR_RIGHT);

    // Wheel velocities in cm/s
    float lf_vel = robot->getFrontLeftWheelSpeed()  * 100.0f;
    float lr_vel = robot->getRearLeftWheelSpeed()   * 100.0f;
    float rf_vel = robot->getFrontRightWheelSpeed() * 100.0f;
    float rr_vel = robot->getRearRightWheelSpeed()  * 100.0f;

    float pan_pos  = panTilt ? panTilt->getPan()  : 0.0f;
    float tilt_pos = panTilt ? panTilt->getTilt() : 0.0f;

    Serial.printf("STATE,%lld,%lld,%lld,%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                  lf_pos, lr_pos, rf_pos, rr_pos,
                  lf_vel, lr_vel, rf_vel, rr_vel,
                  pan_pos, tilt_pos);
}

void processSerialCommand(const String& cmd) {
    if (cmd == "GET") {
        sendStateMessage();
    }
    else if (cmd.startsWith("CMD,")) {
        float lf_vel, lr_vel, rf_vel, rr_vel, pan_pos, tilt_pos;
        if (sscanf(cmd.c_str(), "CMD,%f,%f,%f,%f,%f,%f",
                   &lf_vel, &lr_vel, &rf_vel, &rr_vel, &pan_pos, &tilt_pos) == 6) {
            // Serial link is alive — feed the watchdog even while e-stopped
            lastMotorCommandMs = millis();
            g_watchdogBrake = false;

            // Convert cm/s to m/s, average per side for left/right differential drive
            if (!g_eStopActive) {
                float leftMPS  = (lf_vel + lr_vel) / 2.0f / 100.0f;
                float rightMPS = (rf_vel + rr_vel) / 2.0f / 100.0f;
                if (!robot->isEnabled()) robot->enable(true);
                robot->setWheelSpeeds(leftMPS, rightMPS);
            }

            if (panTilt) {
                panTilt->setPan(pan_pos);
                panTilt->setTilt(tilt_pos);
            }
        }
    }
    else if (cmd.startsWith("AUX,")) {
        // Parse AUX,command_name,arg
        String rest = cmd.substring(4);
        int comma = rest.indexOf(',');
        String command_name, arg;
        if (comma >= 0) {
            command_name = rest.substring(0, comma);
            arg = rest.substring(comma + 1);
        } else {
            command_name = rest;
            arg = "";
        }
        if (command_name == "toggle") {
            static bool auxPinState = false;
            auxPinState = !auxPinState;
            g_auxPinHigh = auxPinState;
            digitalWrite(PIN_AUX_OUT, auxPinState ? HIGH : LOW);
        } else if (command_name == "fire") {
            if (!g_auxPinHigh) {
                g_auxPinHigh = true;
                digitalWrite(PIN_AUX_OUT, HIGH);
                TimerHandle_t t = xTimerCreate("auxOff", pdMS_TO_TICKS(600), pdFALSE, nullptr,
                    [](TimerHandle_t xTimer) {
                        digitalWrite(PIN_AUX_OUT, LOW);
                        g_auxPinHigh = false;
                        xTimerDelete(xTimer, 0);
                    });
                if (t) xTimerStart(t, 0);
            }
        } else if (command_name == "estop") {
            g_eStopActive = true;   // motorControlTask performs the disable+brake
        } else if (command_name == "estopclear") {
            g_eStopActive = false;  // motorControlTask re-enables when it sees this
        }
    }
}

void serialCommandTask(void* parameter) {
    String line = "";
    line.reserve(64);

    while (true) {
        // Watchdog: request a brake if no CMD within timeout. The brake
        // itself is performed by motorControlTask (single writer of motor
        // state); the next CMD clears the flag and resumes normal control.
        uint32_t lastCmd = lastMotorCommandMs;
        if (lastCmd > 0 && (millis() - lastCmd) > SERIAL_WATCHDOG_MS) {
            lastMotorCommandMs = 0;  // reset to prevent repeated triggers
            g_watchdogBrake = true;
        }

        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\r' || c == '\n') {
                if (line.length() > 0) {
                    processSerialCommand(line);
                    line = "";
                }
            } else {
                line += c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// FREERTOS TASKS
// ============================================================================

void motorControlTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(MOTOR_UPDATE_INTERVAL_MS);

    Serial.println("[Motor Task] Started");

    bool wasBraked = false;

    while (true) {
        // E-stop / watchdog braking happens here, not in the tasks that
        // request it, so the brake can never be overwritten by an
        // in-flight PID cycle. Re-asserted every cycle while active.
        // update() below is a no-op while disabled, and encoder updates
        // continue so reported velocities stay live.
        bool braked = g_eStopActive || g_watchdogBrake;
        if (braked) {
            if (robot->isEnabled()) robot->enable(false);
            robot->brake();
        } else if (wasBraked) {
            robot->enable(true);  // resume after estopclear / comms restored
        }
        wasBraked = braked;

        // Check individual motor faults
        bool fault = false;
        for (int i = 0; i < 4; i++) {
            if (motors[i]->hasFault()) {
                // Runtime print — gated so it can't interleave with STATE
                // lines on the protocol UART (see DebugLog.h)
                DBG_PRINTF("[Motor Task] Motor %d FAULT (code %d)\n", i + 1, motors[i]->faultCode());
                fault = true;
            }
        }

        if (fault) {
            robot->brake();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Update encoder velocities
        for (int i = 0; i < 4; i++) {
            motors[i]->updateEncoder();
        }

        // Update robot controller (applies motor commands)
        robot->update();

        // Wait for next cycle
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

void telemetryTask(void* parameter) {
    const TickType_t interval = pdMS_TO_TICKS(TELEMETRY_UPDATE_INTERVAL_MS);

    Serial.println("[Telemetry Task] Started");

    while (true) {
        // Update telemetry data with mutex protection
        if (xSemaphoreTake(telemetry.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i = 0; i < 4; i++) {
                telemetry.encoderCounts[i] = motors[i]->getEncoderCount();
                telemetry.motorCurrents[i] = motors[i]->readCurrentAmps();
            }
            telemetry.actualSpeed = robot->getActualLinearSpeed();
            xSemaphoreGive(telemetry.mutex);
        }

        vTaskDelay(interval);
    }
}

// ============================================================================
// SETUP AND MAIN
// ============================================================================

void setup() {
    Serial.begin(115200);
    pinMode(PIN_AUX_OUT, OUTPUT);
    digitalWrite(PIN_AUX_OUT, LOW);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("  ESP32-S3 Quad Motor Robot Controller");
    Serial.println("========================================\n");

    // Initialize telemetry mutex
    telemetry.mutex = xSemaphoreCreateMutex();

    // Initialize MCP23017
    Serial.print("Initializing MCP23017... ");
    if (!mcp.begin(PIN_SDA, PIN_SCL, MCP_ADDR)) {
        Serial.println("FAILED!");
        while (1) { delay(1000); }
    }
    Serial.println("OK");

    // Configure status LEDs (red=off, green=on to indicate ready)
    mcp.pinMode(MCP_RED_LED,   OUTPUT);
    mcp.pinMode(MCP_GREEN_LED, OUTPUT);
    mcp.writePin(MCP_RED_LED,   LOW);
    mcp.writePin(MCP_GREEN_LED, HIGH);

    // mcp.pinMode(0,   OUTPUT);
    // mcp.pinMode(3, OUTPUT);
    // mcp.writePin(0,   LOW);
    // mcp.writePin(3, HIGH);

    // Serial.println("MCP23017 initialized and LEDs configured");
    // delay(50000);
    // Serial.println("Continuing with motor initialization...");

    // Create motor objects
    motors[0] = new Motor(mcp, M1_PINS, MOTOR_CONFIG);
    motors[1] = new Motor(mcp, M2_PINS, MOTOR_CONFIG);
    motors[2] = new Motor(mcp, M3_PINS, MOTOR_CONFIG);
    motors[3] = new Motor(mcp, M4_PINS, MOTOR_CONFIG);

    // Initialize each motor
    for (int i = 0; i < 4; i++) {
        Serial.printf("Initializing Motor %d... ", i + 1);
        if (!motors[i]->begin(static_cast<pcnt_unit_t>(i))) {
            Serial.println("FAILED!");
            while (1) { delay(1000); }
        }
        Serial.println("OK");
    }

    // Create robot controller
    robot = new RobotController(motors, ROBOT_PARAMS, &mcp, MCP_RED_LED, MCP_GREEN_LED);
    robot->begin();
    Serial.println("Robot controller initialized");

    // Initialize pan/tilt controller (PCA9685 on same I2C bus)
    panTilt = new PanTiltController();
    if (!panTilt->begin()) {
        Serial.println("Pan/tilt init failed — continuing without it");
        delete panTilt;
        panTilt = nullptr;
    }

    // Print robot parameters
    Serial.println("\nRobot Parameters:");
    Serial.printf("  Wheel diameter: %.3f m\n", ROBOT_PARAMS.wheelDiameterM);
    Serial.printf("  Encoder CPR: %.0f counts/rev\n", ROBOT_PARAMS.encoderCPR);
    Serial.printf("  Wheel base: %.3f m\n", ROBOT_PARAMS.wheelBaseM);
    Serial.printf("  Max speed: %.2f m/s\n", ROBOT_PARAMS.maxSpeedMPS);

    // Connect to WiFi and start web server
    Serial.println("\nConnecting to WiFi...");
    dashboard = new WebDashboard(*robot, panTilt);
    if (!dashboard->begin(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("WiFi failed - continuing without web dashboard");
    } else {
        // Start mDNS responder
        if (MDNS.begin(MDNS_HOSTNAME)) {
            MDNS.addService("http", "tcp", 80);
            Serial.printf("mDNS started: http://%s.local\n", MDNS_HOSTNAME);
        } else {
            Serial.println("mDNS failed to start");
        }

        Serial.println("\n========================================");
        Serial.print("  Dashboard: http://");
        Serial.println(dashboard->getIP());
        Serial.println("========================================\n");
    }

    // ---- TEST: Drive forward for 5 seconds ----
    // Serial.println("\n[TEST] Driving forward at 0.3 m/s for 5 seconds...");
    // bool prevPIDState = robot->isPIDEnabled();
    // bool tmp = robot->isEnabled();
    // robot->enable(true);
    // robot->forward(0.3f);
    
    // robot->enablePID(false);
    // uint32_t testStart = millis();
    // while (millis() - testStart < 5000) {
    //     for (int i = 0; i < 4; i++) motors[i]->updateEncoder();
    //     robot->update();
    //     delay(10);
    // }
    // robot->brake();
    // robot->enablePID(prevPIDState);
    // robot->enable(tmp);
    // Serial.println("[TEST] Done.\n");
    // ---- END TEST ----

    // Create FreeRTOS tasks
    Serial.println("Starting FreeRTOS tasks...");

    xTaskCreatePinnedToCore(
        motorControlTask,
        "MotorControl",
        MOTOR_TASK_STACK_SIZE,
        nullptr,
        MOTOR_TASK_PRIORITY,
        &motorTaskHandle,
        1  // Run on core 1
    );

    xTaskCreatePinnedToCore(
        telemetryTask,
        "Telemetry",
        TELEMETRY_TASK_STACK_SIZE,
        nullptr,
        TELEMETRY_TASK_PRIORITY,
        &telemetryTaskHandle,
        0  // Run on core 0
    );

    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        SERIAL_CMD_TASK_STACK_SIZE,
        nullptr,
        SERIAL_CMD_TASK_PRIORITY,
        &serialCmdTaskHandle,
        0  // Run on core 0
    );

    Serial.println("System ready!\n");
}

void loop() {
    // Main loop is mostly idle - work is done in FreeRTOS tasks
    // Just print status periodically

    static uint32_t lastStatusTime = 0;
    uint32_t now = millis();

    // Status print disabled — Serial is used for ROS2 serial protocol
    (void)lastStatusTime;
    (void)now;

    delay(100);
}
