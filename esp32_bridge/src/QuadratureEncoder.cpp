#include "QuadratureEncoder.h"

// Static member initialization
QuadratureEncoder* QuadratureEncoder::_instances[MAX_ENCODERS] = {nullptr, nullptr, nullptr, nullptr};
bool QuadratureEncoder::_isrInstalled = false;

QuadratureEncoder::QuadratureEncoder()
    : _pinA(0)
    , _pinB(0)
    , _unit(PCNT_UNIT_0)
    , _initialized(false)
    , _overflowCount(0)
    , _lastCount(0)
    , _lastUpdateTime(0)
    , _velocity(0)
{
}

QuadratureEncoder::~QuadratureEncoder() {
    if (_initialized) {
        pcnt_counter_pause(_unit);
        pcnt_isr_handler_remove(_unit);
        _instances[_unit] = nullptr;
    }
}

void IRAM_ATTR QuadratureEncoder::pcntOverflowHandler(void* arg) {
    pcnt_unit_t unit = *static_cast<pcnt_unit_t*>(arg);
    QuadratureEncoder* enc = _instances[unit];

    if (enc != nullptr) {
        uint32_t status = 0;
        pcnt_get_event_status(unit, &status);

        if (status & PCNT_EVT_H_LIM) {
            enc->_overflowCount += 32767;
        }
        if (status & PCNT_EVT_L_LIM) {
            enc->_overflowCount -= 32768;
        }
    }
}

bool QuadratureEncoder::begin(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit) {
    if (unit >= MAX_ENCODERS) {
        return false;
    }

    _pinA = pinA;
    _pinB = pinB;
    _unit = unit;

    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = static_cast<int>(_pinA),
        .ctrl_gpio_num = static_cast<int>(_pinB),
        .lctrl_mode = PCNT_MODE_KEEP,    // Reverse counting direction if ctrl is low
        .hctrl_mode = PCNT_MODE_REVERSE,       // Keep counting direction if ctrl is high
        .pos_mode = PCNT_COUNT_INC,         // Count up on rising edge
        .neg_mode = PCNT_COUNT_DEC,         // Count down on falling edge
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = _unit,
        .channel = PCNT_CHANNEL_0,
    };

    esp_err_t err = pcnt_unit_config(&pcnt_config);
    if (err != ESP_OK) {
        return false;
    }

    // Configure second channel for full quadrature resolution
    pcnt_config.pulse_gpio_num = static_cast<int>(_pinB);
    pcnt_config.ctrl_gpio_num = static_cast<int>(_pinA);
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

    err = pcnt_unit_config(&pcnt_config);
    if (err != ESP_OK) {
        return false;
    }

    // Configure and enable input filter (13 APB clock cycles)
    pcnt_set_filter_value(_unit, 100);
    pcnt_filter_enable(_unit);

    // Enable events for overflow handling
    pcnt_event_enable(_unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(_unit, PCNT_EVT_L_LIM);

    // Install ISR service if not already installed
    if (!_isrInstalled) {
        err = pcnt_isr_service_install(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return false;
        }
        _isrInstalled = true;
    }

    // Store instance pointer and add ISR handler
    _instances[_unit] = this;

    static pcnt_unit_t unitArgs[MAX_ENCODERS] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};
    err = pcnt_isr_handler_add(_unit, pcntOverflowHandler, &unitArgs[_unit]);
    if (err != ESP_OK) {
        _instances[_unit] = nullptr;
        return false;
    }

    // Initialize counter
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);

    _overflowCount = 0;
    _lastCount = 0;
    _lastUpdateTime = micros();
    _velocity = 0;
    _initialized = true;

    return true;
}

int64_t QuadratureEncoder::getCount() const {
    if (!_initialized) return 0;

    int16_t count = 0;
    pcnt_get_counter_value(_unit, &count);

    return _overflowCount + count;
}

void QuadratureEncoder::resetCount() {
    if (!_initialized) return;

    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    _overflowCount = 0;
    _lastCount = 0;
    pcnt_counter_resume(_unit);
}

void QuadratureEncoder::setCount(int64_t count) {
    if (!_initialized) return;

    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    _overflowCount = count;
    _lastCount = count;
    pcnt_counter_resume(_unit);
}

bool QuadratureEncoder::getDirection() const {
    if (!_initialized) return true;

    // Compare current count with last count
    int64_t current = getCount();
    return current >= _lastCount;
}

float QuadratureEncoder::getVelocity() const {
    return _velocity;
}

void QuadratureEncoder::update() {
    if (!_initialized) return;

    uint32_t now = micros();
    uint32_t dt = now - _lastUpdateTime;

    if (dt > 0) {
        int64_t currentCount = getCount();
        int64_t deltaCount = currentCount - _lastCount;

        // Calculate velocity in counts per second
        _velocity = (float)deltaCount * 1000000.0f / (float)dt;

        _lastCount = currentCount;
        _lastUpdateTime = now;
    }
}
