#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Mcp23017Bus.h"
#include "QuadratureEncoder.h"

// Pin mapping structure
struct MotorPins {
    uint8_t mcpINA;         // MCP23017 pin for INA (0-15)
    uint8_t mcpINB;         // MCP23017 pin for INB (0-15)
    uint8_t mcpSEL0;        // MCP23017 pin for SEL0 (shared MultiSense select)
    uint8_t mcpSEL1;        // MCP23017 pin for SEL1 (shared MultiSense select)

    uint8_t pwmGPIO;        // ESP32 GPIO for PWM output
    uint8_t csAdcGPIO;      // ESP32 GPIO for current sense ADC

    uint8_t encA;           // ESP32 GPIO for encoder channel A
    uint8_t encB;           // ESP32 GPIO for encoder channel B
};

// Motor configuration structure
struct MotorConfig {
    uint32_t pwmFreqHz = 20000;           // PWM frequency in Hz
    uint8_t pwmResolutionBits = 10;       // PWM resolution (10 = 0-1023)
    bool invertDirection = false;         // Invert motor direction
    float csVoltsPerAmp = 0.14f;          // Current sense scaling (V/A) - calibrate for VNH7040
    float csZeroOffset = 0.0f;            // Current sense zero offset voltage
    float faultVoltageThreshold = 3.0f;   // VNH7040: fault when MultiSense exceeds this (VsenseH)
};

// Fault codes
enum MotorFaultCode {
    FAULT_NONE = 0,
    FAULT_DIAG_PIN = 1,
    FAULT_GLOBAL_LINE = 2,
    FAULT_OVERCURRENT = 3
};

class Motor {
public:
    Motor(Mcp23017Bus& mcp, const MotorPins& pins, const MotorConfig& config);

    // Initialize motor (call after MCP23017 is initialized)
    bool begin(pcnt_unit_t encoderUnit = PCNT_UNIT_0);

    // Enable/disable motor driver
    void enable(bool en);
    bool isEnabled() const;

    // Set duty cycle (-1.0 to +1.0, negative = reverse)
    void setDuty(float duty);

    // Set raw PWM value (0 to max based on resolution)
    void setPWM(uint16_t pwm);

    // Set direction (true = forward, false = reverse)
    void setDirection(bool forward);

    // Get current direction
    bool getDirection() const { return _currentDirection; }

    // Active braking (short motor terminals)
    void brake();

    // Coast (disable outputs)
    void coast();

    // Read raw ADC value from current sense
    uint16_t readCurrentRaw();

    // Read current in amps
    float readCurrentAmps();

    // Get encoder count
    int64_t getEncoderCount();

    // Reset encoder count to zero
    void resetEncoderCount();

    // Get encoder velocity (counts/sec)
    float getEncoderVelocity();

    // Update encoder velocity (call periodically)
    void updateEncoder();

    // Check if motor has fault
    bool hasFault();

    // Get fault code
    MotorFaultCode faultCode();

    // Clear fault state
    void clearFault();

    // Get current duty cycle
    float getDuty() const { return _currentDuty; }

    // Get current PWM value
    uint16_t getPWM() const { return _currentPWM; }

private:
    void applyDirection(bool forward);
    void applyPWM(uint16_t pwm);
    void setSEL(bool forwardSide);

    Mcp23017Bus& _mcp;
    MotorPins _pins;
    MotorConfig _config;
    QuadratureEncoder _encoder;

    uint8_t _ledcChannel;
    uint16_t _maxPWM;

    bool _enabled;
    bool _currentDirection;
    float _currentDuty;
    uint16_t _currentPWM;
    MotorFaultCode _faultCode;

    static uint8_t _nextLedcChannel;
};

#endif // MOTOR_H
