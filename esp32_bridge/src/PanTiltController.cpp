#include "PanTiltController.h"

PanTiltController::PanTiltController(const PanTiltConfig& config)
    : _config(config)
    , _pwm(config.i2cAddress)
{
}

bool PanTiltController::begin() {
    if (!_pwm.begin()) {
        Serial.println("[PanTilt] PCA9685 not found on I2C bus");
        return false;
    }
    _pwm.setOscillatorFrequency(27000000);  // Trim for accurate pulse widths
    _pwm.setPWMFreq(_config.servoFreqHz);
    delay(10);

    center();
    Serial.println("[PanTilt] PCA9685 initialised");
    return true;
}

void PanTiltController::setPan(float radians) {
    _panRad = constrain(radians, _config.panMinRad, _config.panMaxRad);
    writeServo(_config.panChannel, _panRad, _config.panMinRad, _config.panMaxRad);
}

void PanTiltController::setTilt(float radians) {
    _tiltRad = constrain(radians, _config.tiltMinRad, _config.tiltMaxRad);
    writeServo(_config.tiltChannel, _tiltRad, _config.tiltMinRad, _config.tiltMaxRad);
}

void PanTiltController::center() {
    setPan(0.0f);
    setTilt(0.0f);
}

void PanTiltController::writeServo(uint8_t channel, float radians, float minRad, float maxRad) {
    float normalised = (radians - minRad) / (maxRad - minRad);  // 0.0 – 1.0
    uint16_t pulseUs = (uint16_t)(_config.pulseMinUs
                       + normalised * (_config.pulseMaxUs - _config.pulseMinUs));
    _pwm.writeMicroseconds(channel, pulseUs);
}
