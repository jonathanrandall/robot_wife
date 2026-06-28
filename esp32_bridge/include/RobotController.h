#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <Arduino.h>
#include "Motor.h"
#include "PIDController.h"
#include "Mcp23017Bus.h"

// Robot physical parameters (configurable)
struct RobotParams {
    float wheelDiameterM = 0.090f;      // Wheel diameter in meters (90mm default)
    float encoderCPR = 537.7f;          // Encoder counts per revolution
    float wheelBaseM = 0.20f;           // Distance between left and right wheels in meters
    float maxSpeedMPS = 1.5f;           // Maximum speed in meters per second
};

// PID configuration for speed control
// Note: PID outputs a CORRECTION to the feedforward duty cycle
struct SpeedPIDConfig {
    float Kp = 0.5f;            // Proportional gain (correction per m/s error)
    float Ki = 0.05f;            // Integral gain (eliminates steady-state error)
    float Kd = 0.01f;           // Derivative gain (reduces overshoot)
    float maxIntegral = 0.3f;   // Anti-windup limit for correction
    float maxCorrection = 0.5f; // Maximum PID correction (+/- this value)
};

// Motor indices for a 4-wheel differential drive robot
// M1=index 0 (rear right), M2=index 1 (rear left),
// M3=index 2 (front left), M4=index 3 (front right)
enum MotorPosition {
    FRONT_LEFT  = 2,
    FRONT_RIGHT = 3,
    REAR_LEFT   = 1,
    REAR_RIGHT  = 0
};

// Robot movement commands
struct RobotCommand {
    float linearSpeed;      // m/s, positive = forward
    float angularSpeed;     // rad/s, positive = turn left
};

class RobotController {
public:
    RobotController(Motor* motors[4], const RobotParams& params,
                    Mcp23017Bus* mcp = nullptr,
                    uint8_t redLedPin = 0, uint8_t greenLedPin = 0);

    // Initialize the controller
    void begin();

    // Set robot parameters
    void setParams(const RobotParams& params);
    const RobotParams& getParams() const { return _params; }

    // Movement commands
    void setSpeed(float linearMPS, float angularRadPS);
    void setWheelSpeeds(float leftMPS, float rightMPS);  // direct left/right target — for ROS2 serial interface
    void forward(float speedMPS);
    void backward(float speedMPS);
    void turnLeft(float speedMPS);
    void turnRight(float speedMPS);
    void stop();
    void brake();

    // Get current command
    const RobotCommand& getCommand() const { return _command; }

    // Convert between speed units
    float mpsToEncoderCPS(float mps) const;     // m/s to encoder counts/sec
    float encoderCPSToMPS(float cps) const;     // encoder counts/sec to m/s
    float mpsToDuty(float mps) const;           // m/s to duty cycle (-1 to 1)

    // Update motor outputs (call from FreeRTOS task)
    // dt: time since last update in seconds
    void update(float dt);
    void update();  // Uses internal timing

    // Get actual speeds from encoders
    float getActualLinearSpeed() const;
    float getActualAngularSpeed() const;
    float getLeftWheelSpeed() const;
    float getRightWheelSpeed() const;
    float getFrontLeftWheelSpeed() const;
    float getRearLeftWheelSpeed() const;
    float getFrontRightWheelSpeed() const;
    float getRearRightWheelSpeed() const;

    // Get motor data for telemetry
    int64_t getEncoderCount(int motorIndex) const;
    float getMotorCurrent(int motorIndex) const;
    uint16_t getMotorPWM(int motorIndex) const;
    float getMotorDuty(int motorIndex) const;

    // Enable/disable motors
    void enable(bool en);
    bool isEnabled() const { return _enabled; }

    // LED control
    void setLEDs(bool red, bool green);

    // PID configuration
    void setPIDGains(const SpeedPIDConfig& config);
    void setPIDGains(float Kp, float Ki, float Kd);
    const SpeedPIDConfig& getPIDConfig() const { return _pidConfig; }

    // Enable/disable PID control (false = open-loop)
    void enablePID(bool enable) { _pidEnabled = enable; }
    bool isPIDEnabled() const { return _pidEnabled; }

    // Reset PID controllers (call when stopped or re-enabled)
    void resetPID();

    // Get PID debug info
    float getFrontLeftPIDOutput() const  { return _frontLeftPID.getOutput(); }
    float getRearLeftPIDOutput() const   { return _rearLeftPID.getOutput(); }
    float getFrontRightPIDOutput() const { return _frontRightPID.getOutput(); }
    float getRearRightPIDOutput() const  { return _rearRightPID.getOutput(); }
    float getFrontLeftPIDError() const   { return _frontLeftPID.getError(); }
    float getRearLeftPIDError() const    { return _rearLeftPID.getError(); }
    float getFrontRightPIDError() const  { return _frontRightPID.getError(); }
    float getRearRightPIDError() const   { return _rearRightPID.getError(); }

private:
    Motor** _motors;
    RobotParams _params;
    Mcp23017Bus* _mcp;
    uint8_t _redLedPin;
    uint8_t _greenLedPin;
    RobotCommand _command;
    bool _enabled;

    // Wheel circumference cache
    float _wheelCircumference;

    // PID controllers — one per wheel
    PIDController _frontLeftPID;
    PIDController _rearLeftPID;
    PIDController _frontRightPID;
    PIDController _rearRightPID;
    SpeedPIDConfig _pidConfig;
    bool _pidEnabled;

    // Timing for update()
    uint32_t _lastUpdateTime;

    // Target wheel speeds (m/s) — shared per side
    float _targetLeftSpeed;
    float _targetRightSpeed;

    // Per-wheel duty cycles
    float _leftFrontDuty  = 0;
    float _leftBackDuty   = 0;
    float _rightFrontDuty = 0;
    float _rightBackDuty  = 0;

    void updateWheelCircumference();
    void applyPIDControl(float dt);
    void applyOpenLoopControl();
};

#endif // ROBOTCONTROLLER_H
