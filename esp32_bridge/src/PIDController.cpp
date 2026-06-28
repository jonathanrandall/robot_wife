#include "PIDController.h"

PIDController::PIDController()
    : _gains{1.0f, 0.1f, 0.01f, 1.0f, 1.0f, -1.0f}
    , _integral(0)
    , _lastError(0)
    , _lastMeasurement(0)
    , _firstRun(true)
    , _lastP(0)
    , _lastD(0)
    , _lastOutput(0)
    , _pEnabled(true)
    , _iEnabled(true)
    , _dEnabled(true)
{
}

PIDController::PIDController(const PIDGains& gains)
    : _gains(gains)
    , _integral(0)
    , _lastError(0)
    , _lastMeasurement(0)
    , _firstRun(true)
    , _lastP(0)
    , _lastD(0)
    , _lastOutput(0)
    , _pEnabled(true)
    , _iEnabled(true)
    , _dEnabled(true)
{
}

void PIDController::setGains(const PIDGains& gains) {
    _gains = gains;
}

void PIDController::setGains(float Kp, float Ki, float Kd) {
    _gains.Kp = Kp;
    _gains.Ki = Ki;
    _gains.Kd = Kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _gains.minOutput = min;
    _gains.maxOutput = max;
}

void PIDController::setMaxIntegral(float maxIntegral) {
    _gains.maxIntegral = maxIntegral;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0) {
        return _lastOutput;
    }

    // Calculate error
    float error = setpoint - measurement;

    // Proportional term
    _lastP = 0;
    if (_pEnabled) {
        _lastP = _gains.Kp * error;
    }

    // Integral term with anti-windup
    if (_iEnabled && dt > 0) {
        _integral += error * dt;
        // Anti-windup: clamp integral
        _integral = clamp(_integral, -_gains.maxIntegral, _gains.maxIntegral);
    }
    float iTerm = _gains.Ki * _integral;

    // Derivative term (on measurement to avoid derivative kick on setpoint change)
    _lastD = 0;
    if (_dEnabled && !_firstRun && dt > 0) {
        // Derivative on measurement (negative because we want d(error)/dt)
        float dMeasurement = (measurement - _lastMeasurement) / dt;
        _lastD = -_gains.Kd * dMeasurement;
    }

    // Calculate total output
    float output = _lastP + iTerm + _lastD;

    // Clamp output
    output = clamp(output, _gains.minOutput, _gains.maxOutput);

    // Store values for next iteration
    _lastError = error;
    _lastMeasurement = measurement;
    _lastOutput = output;
    _firstRun = false;

    return output;
}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
    _lastMeasurement = 0;
    _firstRun = true;
    _lastP = 0;
    _lastD = 0;
    _lastOutput = 0;
}

float PIDController::clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
