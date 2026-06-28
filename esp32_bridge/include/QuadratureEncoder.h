#ifndef QUADRATUREENCODER_H
#define QUADRATUREENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h"

// Maximum number of encoder instances (ESP32-S3 has 4 PCNT units)
#define MAX_ENCODERS 4

class QuadratureEncoder {
public:
    QuadratureEncoder();
    ~QuadratureEncoder();

    // Initialize encoder with specified pins and PCNT unit
    bool begin(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit = PCNT_UNIT_0);

    // Get current position (64-bit to handle overflow)
    int64_t getCount() const;

    // Reset position counter to zero
    void resetCount();

    // Set current count value
    void setCount(int64_t count);

    // Get direction (true = positive/forward, false = negative/reverse)
    bool getDirection() const;

    // Get velocity in counts per second (requires periodic update() calls)
    float getVelocity() const;

    // Update velocity calculation (call periodically, e.g., every 10-50ms)
    void update();

    // Check if encoder is initialized
    bool isInitialized() const { return _initialized; }

    // Get the PCNT unit being used
    pcnt_unit_t getUnit() const { return _unit; }

private:
    static void IRAM_ATTR pcntOverflowHandler(void* arg);

    uint8_t _pinA;
    uint8_t _pinB;
    pcnt_unit_t _unit;
    bool _initialized;

    volatile int64_t _overflowCount;    // Accumulated overflow counts
    int64_t _lastCount;                 // Last count for velocity calc
    uint32_t _lastUpdateTime;           // Last update time in micros
    float _velocity;                    // Calculated velocity

    static QuadratureEncoder* _instances[MAX_ENCODERS];
    static bool _isrInstalled;
};

#endif // QUADRATUREENCODER_H
