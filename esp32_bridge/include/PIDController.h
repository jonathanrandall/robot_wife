#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>

// PID tuning parameters
struct PIDGains {
    float Kp = 1.0f;        // Proportional gain
    float Ki = 0.1f;        // Integral gain
    float Kd = 0.01f;       // Derivative gain
    float maxIntegral = 1.0f;   // Anti-windup limit for integral term
    float maxOutput = 1.0f;     // Maximum output value
    float minOutput = -1.0f;    // Minimum output value
};

class PIDController {
public:
    PIDController();
    PIDController(const PIDGains& gains);

    // Set PID gains
    void setGains(const PIDGains& gains);
    void setGains(float Kp, float Ki, float Kd);
    const PIDGains& getGains() const { return _gains; }

    // Set output limits
    void setOutputLimits(float min, float max);

    // Set integral anti-windup limit
    void setMaxIntegral(float maxIntegral);

    // Compute PID output
    // setpoint: desired value
    // measurement: actual measured value
    // dt: time delta in seconds
    // Returns: control output (clamped to output limits)
    float compute(float setpoint, float measurement, float dt);

    // Reset controller state (call when re-enabling or changing setpoint significantly)
    void reset();

    // Get individual terms for debugging/tuning
    float getProportional() const { return _lastP; }
    float getIntegral() const { return _integral; }
    float getDerivative() const { return _lastD; }
    float getError() const { return _lastError; }
    float getOutput() const { return _lastOutput; }

    // Enable/disable individual terms
    void enableProportional(bool enable) { _pEnabled = enable; }
    void enableIntegral(bool enable) { _iEnabled = enable; }
    void enableDerivative(bool enable) { _dEnabled = enable; }

private:
    PIDGains _gains;

    float _integral;
    float _lastError;
    float _lastMeasurement;
    bool _firstRun;

    // Last computed values for debugging
    float _lastP;
    float _lastD;
    float _lastOutput;

    // Term enable flags
    bool _pEnabled;
    bool _iEnabled;
    bool _dEnabled;

    float clamp(float value, float min, float max);
};

#endif // PIDCONTROLLER_H
