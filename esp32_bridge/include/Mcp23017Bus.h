#ifndef MCP23017BUS_H
#define MCP23017BUS_H

#include <Arduino.h>
#include <Wire.h>

// MCP23017 Register addresses (BANK=0 mode)
#define MCP23017_IODIRA   0x00
#define MCP23017_IODIRB   0x01
#define MCP23017_IPOLA    0x02
#define MCP23017_IPOLB    0x03
#define MCP23017_GPINTENA 0x04
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALA  0x06
#define MCP23017_DEFVALB  0x07
#define MCP23017_INTCONA  0x08
#define MCP23017_INTCONB  0x09
#define MCP23017_IOCON    0x0A
#define MCP23017_GPPUA    0x0C
#define MCP23017_GPPUB    0x0D
#define MCP23017_INTFA    0x0E
#define MCP23017_INTFB    0x0F
#define MCP23017_INTCAPA  0x10
#define MCP23017_INTCAPB  0x11
#define MCP23017_GPIOA    0x12
#define MCP23017_GPIOB    0x13
#define MCP23017_OLATA    0x14
#define MCP23017_OLATB    0x15

class Mcp23017Bus {
public:
    Mcp23017Bus();

    // Initialize the MCP23017
    bool begin(int sda, int scl, uint8_t addr = 0x20, uint32_t i2cFreq = 400000);

    // Single pin operations (pin 0-15, where 0-7 = PORTA, 8-15 = PORTB)
    void pinMode(uint8_t pin, uint8_t mode);
    void writePin(uint8_t pin, bool value);
    bool readPin(uint8_t pin);

    // Port operations
    void writePortA(uint8_t value);
    void writePortB(uint8_t value);
    uint8_t readPortA();
    uint8_t readPortB();

    // Direct register access
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

    // Configure pin as input with pullup
    void enablePullup(uint8_t pin, bool enable);

    // Flush shadow registers to hardware
    void flush();

    // Get current shadow register values
    uint8_t getShadowA() const { return _shadowA; }
    uint8_t getShadowB() const { return _shadowB; }

private:
    uint8_t _addr;
    uint8_t _shadowA;      // Shadow register for GPIOA outputs
    uint8_t _shadowB;      // Shadow register for GPIOB outputs
    uint8_t _dirA;         // Direction register A (1=input, 0=output)
    uint8_t _dirB;         // Direction register B
    uint8_t _pullupA;      // Pullup register A
    uint8_t _pullupB;      // Pullup register B
    TwoWire* _wire;
    bool _initialized;
};

#endif // MCP23017BUS_H
