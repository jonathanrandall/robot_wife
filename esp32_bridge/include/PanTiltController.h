#ifndef PANTILTCONTROLLER_H
#define PANTILTCONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

struct PanTiltConfig {
    uint8_t  i2cAddress  = 0x40;   // PCA9685 default I2C address
    uint8_t  panChannel  = 7;      // PCA9685 channel for pan servo
    uint8_t  tiltChannel = 4;      // PCA9685 channel for tilt servo
    float    servoFreqHz = 50.0f;  // Standard servo PWM frequency

    // Pulse width limits in microseconds
    uint16_t pulseMinUs  = 500;    // Pulse at minimum angle
    uint16_t pulseMaxUs  = 2500;   // Pulse at maximum angle

    // Angle limits in radians
    float    panMinRad   = -1.5708f;   // -π/2
    float    panMaxRad   =  1.5708f;   // +π/2
    float    tiltMinRad  = -0.7854f;   // -π/4
    float    tiltMaxRad   =  1.5708f;  // +π/2
};

class PanTiltController {
public:
    PanTiltController(const PanTiltConfig& config = PanTiltConfig{});

    // Call after Wire.begin() — shares the existing I2C bus
    bool begin();

    // Set pan/tilt position in radians (clamped to configured limits)
    void setPan(float radians);
    void setTilt(float radians);

    // Return last commanded position
    float getPan()  const { return _panRad; }
    float getTilt() const { return _tiltRad; }

    // Move both servos to centre position
    void center();

    const PanTiltConfig& getConfig() const { return _config; }

private:
    void writeServo(uint8_t channel, float radians, float minRad, float maxRad);

    PanTiltConfig           _config;
    Adafruit_PWMServoDriver _pwm;
    float                   _panRad  = 0.0f;
    float                   _tiltRad = 0.0f;
};

#endif // PANTILTCONTROLLER_H
