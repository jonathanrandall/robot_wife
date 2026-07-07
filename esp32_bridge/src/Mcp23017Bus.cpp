#include "Mcp23017Bus.h"

Mcp23017Bus::Mcp23017Bus()
    : _addr(0x20)
    , _shadowA(0)
    , _shadowB(0)
    , _dirA(0xFF)    // Default all inputs
    , _dirB(0xFF)
    , _pullupA(0)
    , _pullupB(0)
    , _wire(&Wire)
    , _initialized(false)
    , _mutex(nullptr)
{
}

void Mcp23017Bus::lock() {
    if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
}

void Mcp23017Bus::unlock() {
    if (_mutex) xSemaphoreGiveRecursive(_mutex);
}

bool Mcp23017Bus::begin(int sda, int scl, uint8_t addr, uint32_t i2cFreq) {
    _addr = addr;
    _wire = &Wire;

    if (!_mutex) {
        _mutex = xSemaphoreCreateRecursiveMutex();
    }

    // Initialize I2C with specified pins
    _wire->begin(sda, scl);
    _wire->setClock(i2cFreq);

    // Check if device is present
    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() != 0) {
        return false;
    }

    // Configure IOCON register (BANK=0, MIRROR=0, SEQOP=0, HAEN=1)
    writeRegister(MCP23017_IOCON, 0x08);

    // Set all pins as outputs initially (will be overridden per-pin)
    _dirA = 0xFF;
    _dirB = 0xFF;
    writeRegister(MCP23017_IODIRA, _dirA);
    writeRegister(MCP23017_IODIRB, _dirB);

    // Clear output latches
    _shadowA = 0;
    _shadowB = 0;
    writeRegister(MCP23017_OLATA, _shadowA);
    writeRegister(MCP23017_OLATB, _shadowB);

    // Disable pullups
    _pullupA = 0;
    _pullupB = 0;
    writeRegister(MCP23017_GPPUA, _pullupA);
    writeRegister(MCP23017_GPPUB, _pullupB);

    _initialized = true;
    return true;
}

void Mcp23017Bus::pinMode(uint8_t pin, uint8_t mode) {
    if (pin > 15) return;

    bool isInput = (mode == INPUT || mode == INPUT_PULLUP);
    bool pullup = (mode == INPUT_PULLUP);

    lock();
    if (pin < 8) {
        // Port A
        if (isInput) {
            _dirA |= (1 << pin);
        } else {
            _dirA &= ~(1 << pin);
        }
        if (pullup) {
            _pullupA |= (1 << pin);
        } else {
            _pullupA &= ~(1 << pin);
        }
        writeRegister(MCP23017_IODIRA, _dirA);
        writeRegister(MCP23017_GPPUA, _pullupA);
    } else {
        // Port B
        uint8_t bit = pin - 8;
        if (isInput) {
            _dirB |= (1 << bit);
        } else {
            _dirB &= ~(1 << bit);
        }
        if (pullup) {
            _pullupB |= (1 << bit);
        } else {
            _pullupB &= ~(1 << bit);
        }
        writeRegister(MCP23017_IODIRB, _dirB);
        writeRegister(MCP23017_GPPUB, _pullupB);
    }
    unlock();
}

void Mcp23017Bus::writePin(uint8_t pin, bool value) {
    if (pin > 15) return;

    lock();
    if (pin < 8) {
        // Port A
        if (value) {
            _shadowA |= (1 << pin);
        } else {
            _shadowA &= ~(1 << pin);
        }
        writeRegister(MCP23017_OLATA, _shadowA);
    } else {
        // Port B
        uint8_t bit = pin - 8;
        if (value) {
            _shadowB |= (1 << bit);
        } else {
            _shadowB &= ~(1 << bit);
        }
        writeRegister(MCP23017_OLATB, _shadowB);
    }
    unlock();
}

bool Mcp23017Bus::readPin(uint8_t pin) {
    if (pin > 15) return false;

    if (pin < 8) {
        uint8_t val = readRegister(MCP23017_GPIOA);
        return (val & (1 << pin)) != 0;
    } else {
        uint8_t bit = pin - 8;
        uint8_t val = readRegister(MCP23017_GPIOB);
        return (val & (1 << bit)) != 0;
    }
}

void Mcp23017Bus::writePortA(uint8_t value) {
    lock();
    _shadowA = value;
    writeRegister(MCP23017_OLATA, _shadowA);
    unlock();
}

void Mcp23017Bus::writePortB(uint8_t value) {
    lock();
    _shadowB = value;
    writeRegister(MCP23017_OLATB, _shadowB);
    unlock();
}

uint8_t Mcp23017Bus::readPortA() {
    return readRegister(MCP23017_GPIOA);
}

uint8_t Mcp23017Bus::readPortB() {
    return readRegister(MCP23017_GPIOB);
}

void Mcp23017Bus::writeRegister(uint8_t reg, uint8_t value) {
    lock();
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
    unlock();
}

uint8_t Mcp23017Bus::readRegister(uint8_t reg) {
    lock();
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission();

    _wire->requestFrom(_addr, (uint8_t)1);
    uint8_t value = 0;
    if (_wire->available()) {
        value = _wire->read();
    }
    unlock();
    return value;
}

void Mcp23017Bus::enablePullup(uint8_t pin, bool enable) {
    if (pin > 15) return;

    lock();
    if (pin < 8) {
        if (enable) {
            _pullupA |= (1 << pin);
        } else {
            _pullupA &= ~(1 << pin);
        }
        writeRegister(MCP23017_GPPUA, _pullupA);
    } else {
        uint8_t bit = pin - 8;
        if (enable) {
            _pullupB |= (1 << bit);
        } else {
            _pullupB &= ~(1 << bit);
        }
        writeRegister(MCP23017_GPPUB, _pullupB);
    }
    unlock();
}

void Mcp23017Bus::flush() {
    lock();
    writeRegister(MCP23017_OLATA, _shadowA);
    writeRegister(MCP23017_OLATB, _shadowB);
    unlock();
}
