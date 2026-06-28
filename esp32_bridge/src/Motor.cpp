#include "Motor.h"

// Static member initialization
uint8_t Motor::_nextLedcChannel = 0;

Motor::Motor(Mcp23017Bus& mcp, const MotorPins& pins, const MotorConfig& config)
    : _mcp(mcp)
    , _pins(pins)
    , _config(config)
    , _ledcChannel(0)
    , _maxPWM(0)
    , _enabled(false)
    , _currentDirection(true)
    , _currentDuty(0)
    , _currentPWM(0)
    , _faultCode(FAULT_NONE)
{
}

bool Motor::begin(pcnt_unit_t encoderUnit) {
    // Allocate LEDC channel
    _ledcChannel = _nextLedcChannel++;
    if (_ledcChannel >= 8) {
        return false;  // ESP32 has 8 LEDC channels
    }

    // Calculate max PWM value based on resolution
    _maxPWM = (1 << _config.pwmResolutionBits) - 1;

    // Configure LEDC for PWM
    ledcSetup(_ledcChannel, _config.pwmFreqHz, _config.pwmResolutionBits);
    ledcAttachPin(_pins.pwmGPIO, _ledcChannel);
    ledcWrite(_ledcChannel, 0);

    // Configure MCP23017 pins as outputs
    _mcp.pinMode(_pins.mcpINA, OUTPUT);
    _mcp.pinMode(_pins.mcpINB, OUTPUT);
    _mcp.pinMode(_pins.mcpSEL0, OUTPUT);
    _mcp.pinMode(_pins.mcpSEL1, OUTPUT);

    // Configure ADC pin for current sense
    pinMode(_pins.csAdcGPIO, INPUT);
    analogSetPinAttenuation(_pins.csAdcGPIO, ADC_11db);

    // Initialize outputs to safe state (off-state: INA=INB=SEL0=SEL1=0)
    _mcp.writePin(_pins.mcpINA, LOW);
    _mcp.writePin(_pins.mcpINB, LOW);
    _mcp.writePin(_pins.mcpSEL0, LOW);
    _mcp.writePin(_pins.mcpSEL1, LOW);

    // Initialize encoder
    if (!_encoder.begin(_pins.encA, _pins.encB, encoderUnit)) {
        return false;
    }

    _enabled = false;
    _currentDirection = true;
    _currentDuty = 0;
    _currentPWM = 0;
    _faultCode = FAULT_NONE;

    return true;
}

void Motor::enable(bool en) {
    _enabled = en;
    // VNH7040 has no EN pin; driver activates via INA/INB/PWM
}

bool Motor::isEnabled() const {
    return _enabled;
}

void Motor::setDuty(float duty) {
    // Clamp duty cycle to -1.0 to +1.0
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    _currentDuty = duty;

    // Determine direction
    bool forward = duty >= 0;
    if (_config.invertDirection) {
        forward = !forward;
    }

    // Calculate PWM value
    float absDuty = duty < 0 ? -duty : duty;
    uint16_t pwm = (uint16_t)(absDuty * _maxPWM);

    // Apply direction and PWM
    applyDirection(forward);
    applyPWM(pwm);
}

void Motor::setPWM(uint16_t pwm) {
    if (pwm > _maxPWM) {
        pwm = _maxPWM;
    }
    _currentPWM = pwm;
    applyPWM(pwm);
}

void Motor::setDirection(bool forward) {
    if (_config.invertDirection) {
        forward = !forward;
    }
    applyDirection(forward);
}

void Motor::brake() {
    // High-side brake: INA=1, INB=1 connects both outputs to VCC
    _mcp.writePin(_pins.mcpINA, HIGH);
    _mcp.writePin(_pins.mcpINB, HIGH);
    ledcWrite(_ledcChannel, _maxPWM);

    _currentDuty = 0;
    _currentPWM = 0;
}

void Motor::coast() {
    // Off-state: INA=INB=0, PWM=0 → outputs float (VNH7040 off-state)
    _mcp.writePin(_pins.mcpINA, LOW);
    _mcp.writePin(_pins.mcpINB, LOW);
    ledcWrite(_ledcChannel, 0);

    _currentDuty = 0;
    _currentPWM = 0;
    _enabled = false;
}

uint16_t Motor::readCurrentRaw() {
    return analogRead(_pins.csAdcGPIO);
}

float Motor::readCurrentAmps() {
    // Set SEL0/SEL1 for MultiSense current output based on active direction
    setSEL(_currentDirection);

    uint16_t raw = readCurrentRaw();
    // Serial.printf("[CS] GPIO%d raw=%d voltage=%.3fV\n", _pins.csAdcGPIO, raw, (raw / 4095.0f) * 3.3f);

    // ESP32-S3 ADC: 12-bit, 0-3.3V with 11dB attenuation
    float voltage = (raw / 4095.0f) * 3.3f;

    // Subtract zero offset
    voltage -= _config.csZeroOffset;

    // Convert to current
    if (_config.csVoltsPerAmp > 0) {
        return voltage / _config.csVoltsPerAmp;
    }

    return 0;
}

int64_t Motor::getEncoderCount() {
    return _encoder.getCount();
}

void Motor::resetEncoderCount() {
    _encoder.resetCount();
}

float Motor::getEncoderVelocity() {
    return _encoder.getVelocity();
}

void Motor::updateEncoder() {
    _encoder.update();
}

bool Motor::hasFault() {
    _faultCode = faultCode();
    return _faultCode != FAULT_NONE;
}

MotorFaultCode Motor::faultCode() {
    // VNH7040: fault indicated by MultiSense rising to VsenseH
    // SEL1=0 is mandatory for fault detection (Table 13)
    setSEL(_currentDirection);

    uint16_t raw = analogRead(_pins.csAdcGPIO);
    float voltage = (raw / 4095.0f) * 3.3f;

    if (voltage >= _config.faultVoltageThreshold) {
        _faultCode = FAULT_DIAG_PIN;
        return FAULT_DIAG_PIN;
    }

    _faultCode = FAULT_NONE;
    return FAULT_NONE;
}

void Motor::clearFault() {
    _faultCode = FAULT_NONE;
}

void Motor::applyDirection(bool forward) {
    _currentDirection = forward;

    if (forward) {
        // Forward: INA=1, INB=0
        _mcp.writePin(_pins.mcpINA, HIGH);
        _mcp.writePin(_pins.mcpINB, LOW);
    } else {
        // Reverse: INA=0, INB=1
        _mcp.writePin(_pins.mcpINA, LOW);
        _mcp.writePin(_pins.mcpINB, HIGH);
    }

    // Readback: verify MCP GPIO register matches shadow
    // bool inaReadback = _mcp.readPin(_pins.mcpINA);
    // bool inbReadback = _mcp.readPin(_pins.mcpINB);
    // Serial.printf("[DIR] INA(pin%d)=%d readback=%d  INB(pin%d)=%d readback=%d\n",
    //               _pins.mcpINA, forward ? 1 : 0, inaReadback,
    //               _pins.mcpINB, forward ? 0 : 1, inbReadback);
}



void Motor::applyPWM(uint16_t pwm) {
    _currentPWM = pwm;
    ledcWrite(_ledcChannel, pwm);
    // Serial.printf("[PWM] GPIO%d ch%d pwm=%d/%d\n", _pins.pwmGPIO, _ledcChannel, pwm, _maxPWM);
}

void Motor::setSEL(bool forwardSide) {
    // Forward (INA=1, INB=0): SEL0=1, SEL1=0 → Current Monitoring HSA (Table 12)
    // Reverse (INA=0, INB=1): SEL0=0, SEL1=0 → Current Monitoring HSB (Table 12)
    _mcp.writePin(_pins.mcpSEL0, forwardSide ? HIGH : LOW);
    _mcp.writePin(_pins.mcpSEL1, LOW);
}
