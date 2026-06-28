# PINMAP_v3.md

# ESP32-S3 + 4x VNH7040AYTR Quad Motor Controller

Last updated: 2026-03-16
Source: KiCad netlist export (kicad-cli sch export netlist)
Verified: GPIO assignments extracted from netlist

---

# System Overview

* MCU: ESP32-S3-WROOM-1 (U1)
* Motor drivers: 4x VNH7040AYTR (U5, U6, U7, U8)
* Direction control: MCP23017-E/SS GPIO expander (U4)
* PWM control: ESP32 LEDC
* Current feedback: M*_CS signals via sense resistors to ESP32 ADC
* Power: AP63200WU-7 buck converter (U3)
* USB: USB-C with USBLC6-2SC6 ESD protection (U2)

---

# ESP32-S3 GPIO Assignment (Verified from Netlist)

## Reserved / System Pins

| Function | GPIO | ESP32 Pin | Notes |
| -------- | ---- | --------- | ----- |
| USB D-   | -    | 13        | USB_D- to USB-C connector |
| USB D+   | -    | 14        | USB_D+ to USB-C connector |
| BOOT     | 0    | 27        | Boot strap, connected to SW1 |
| EN       | -    | 3         | Enable/Reset circuit |

---

## I2C Bus (MCP23017)

| Signal  | Net Name | GPIO | ESP32 Pin |
| ------- | -------- | ---- | --------- |
| I2C_SCL | I2C_SCL  | 15   | 8         |
| I2C_SDA | I2C_SDA  | 16   | 9         |

---

## PWM Outputs to VNH7040

| Motor | Net Name    | GPIO | ESP32 Pin |
| ----- | ----------- | ---- | --------- |
| M1    | M1_PWM_ESP  | 8    | 12        |
| M2    | M2_PWM_ESP  | 48   | 25        |
| M3    | M3_PWM_ESP  | 47   | 24        |
| M4    | M4_PWM_ESP  | 9    | 17        |

---

## Current Sense ADC Inputs

| Motor | Net Name   | GPIO | ESP32 Pin |
| ----- | ---------- | ---- | --------- |
| M1    | M1_CS_ADC  | 4    | 4         |
| M2    | M2_CS_ADC  | 1    | 39        |
| M3    | M3_CS_ADC  | 2    | 38        |
| M4    | M4_CS_ADC  | 5    | 5         |

---

## Encoder Inputs

| Motor | Signal    | Net Name | GPIO | ESP32 Pin |
| ----- | --------- | -------- | ---- | --------- |
| M1    | Encoder A | ENCA_M1  | 17   | 10        |
| M1    | Encoder B | ENCB_M1  | 7    | 7         |
| M2    | Encoder A | ENCA_M2  | 39   | 32        |
| M2    | Encoder B | ENCB_M2  | 38   | 31        |
| M3    | Encoder A | ENCA_M3  | 13   | 21        |
| M3    | Encoder B | ENCB_M3  | 14   | 22        |
| M4    | Encoder A | ENCA_M4  | 11   | 19        |
| M4    | Encoder B | ENCB_M4  | 12   | 20        |

---

## Status LED

| LED | Net Name  | GPIO | ESP32 Pin |
| --- | --------- | ---- | --------- |
| D5  | LED_ESP32 | 42   | 35        |
| D6  | -         | -    | -         |

Note: D6 is orange power LED connected to 3V3 rail, no GPIO control.

---

## Expansion Pins

| Net Name | GPIO | ESP32 Pin | Connector |
| -------- | ---- | --------- | --------- |
| GPIO40   | 40   | 33        | J13 Pin 1 |
| GPIO41   | 41   | 34        | J13 Pin 2 |

---

## Not Connected (no_connect symbols in schematic)

| GPIO | ESP32 Pin | Function |
| ---- | --------- | -------- |
| 3    | 15        | IO3      |
| 6    | 6         | IO6      |
| 10   | 18        | IO10     |
| 18   | 11        | IO18     |
| 21   | 23        | IO21     |
| 35   | 28        | IO35     |
| 36   | 29        | IO36     |
| 37   | 30        | IO37     |
| 45   | 26        | IO45     |
| 46   | 16        | IO46     |
| 43   | 36        | RXD0     |
| 44   | 37        | TXD0     |

---

# MCP23017 GPIO Expander (U4)

Address: 0x20 (A0=A1=A2=GND)

## Port A (GPA0-GPA7)

| Port | Pin | Net Name      | Direction | Function |
| ---- | --- | ------------- | --------- | -------- |
| GPA0 | 21  | M1_INB_ESP32  | OUT       | Motor 1 direction B |
| GPA1 | 22  | Red_LED       | OUT       | Red status LED (D1) |
| GPA2 | 23  | Green_LED     | OUT       | Green status LED (D2) |
| GPA3 | 24  | M1_INA_ESP32  | OUT       | Motor 1 direction A |
| GPA4 | 25  | M4_INB_ESP32  | OUT       | Motor 4 direction B |
| GPA5 | 26  | SEL_0_CTRL    | OUT       | MultiSense SEL0 |
| GPA6 | 27  | SEL_1_CTRL    | OUT       | MultiSense SEL1 |
| GPA7 | 28  | M4_INA_ESP32  | OUT       | Motor 4 direction A |

## Port B (GPB0-GPB7)

| Port | Pin | Net Name      | Direction | Function |
| ---- | --- | ------------- | --------- | -------- |
| GPB0 | 1   | M3_INB_ESP32  | OUT       | Motor 3 direction B |
| GPB1 | 2   | (NC)          | -         | Not connected |
| GPB2 | 3   | (NC)          | -         | Not connected |
| GPB3 | 4   | M3_INA_ESP32  | OUT       | Motor 3 direction A |
| GPB4 | 5   | M2_INB_ESP32  | OUT       | Motor 2 direction B |
| GPB5 | 6   | (NC)          | -         | Not connected |
| GPB6 | 7   | (NC)          | -         | Not connected |
| GPB7 | 8   | M2_INA_ESP32  | OUT       | Motor 2 direction A |

## Configuration

```
IODIRA = 0x00  // All Port A as outputs
IODIRB = 0x00  // All Port B as outputs
```

---

# VNH7040AYTR Motor Driver Connections

## Per Motor Driver

| Signal     | VNH7040 Pin | Source |
| ---------- | ----------- | ------ |
| INA        | 16          | MCP23017 via 1K resistor |
| INB        | 21          | MCP23017 via 1K resistor |
| PWM        | 17          | ESP32 via 1K resistor |
| MultiSense | 19          | To ESP32 ADC via sense network |
| OUTA       | 11-14       | Motor + |
| OUTB       | 23-26       | Motor - |

## MultiSense Selector (Shared)

| Signal | VNH7040 Pin | Source |
| ------ | ----------- | ------ |
| SEL0   | 15          | MCP23017 GPA5 (SEL_0_CTRL) via R7 1K |
| SEL1   | 22          | MCP23017 GPA6 (SEL_1_CTRL) via R16 1K |

## MS_EN (pin 20)

Tied to 3V3 on all drivers.

---

# Motor Direction Truth Table

| INA | INB | Function |
| --- | --- | -------- |
| L   | L   | Brake (low side) |
| H   | L   | Forward |
| L   | H   | Reverse |
| H   | H   | Brake (high side) |

---

# Motor Control Signal Mapping

| Motor | Driver | INA Source      | INB Source      | PWM Source  |
| ----- | ------ | --------------- | --------------- | ----------- |
| M1    | U5     | MCP GPA3 (M1_INA_ESP32) | MCP GPA0 (M1_INB_ESP32) | ESP IO8  |
| M2    | U6     | MCP GPB7 (M2_INA_ESP32) | MCP GPB4 (M2_INB_ESP32) | ESP IO48 |
| M3    | U7     | MCP GPB3 (M3_INA_ESP32) | MCP GPB0 (M3_INB_ESP32) | ESP IO47 |
| M4    | U8     | MCP GPA7 (M4_INA_ESP32) | MCP GPA4 (M4_INB_ESP32) | ESP IO9  |

---

# Current Sense Network

Each motor has a current sense circuit:
- VNH7040 MultiSense (pin 19) -> R_CS* (1K) -> M*_CS net
- M*_CS -> RC filter (C_CS*, 100nF) -> M*_CS_ADC -> ESP32 GPIO

---

# Connectors

## Motor Outputs (J3-J6)

4x 2-pin connectors for motor connections.

## Encoder Inputs (J7-J10)

| Connector | Motor | Pinout |
| --------- | ----- | ------ |
| J7        | M1    | 1:3V3, 2:GND, 3:ENCA_M1, 4:ENCB_M1 |
| J8        | M2    | 1:3V3, 2:GND, 3:ENCA_M2, 4:ENCB_M2 |
| J9        | M3    | 1:3V3, 2:GND, 3:ENCA_M3, 4:ENCB_M3 |
| J10       | M4    | 1:3V3, 2:GND, 3:ENCA_M4, 4:ENCB_M4 |

Note: Encoder signals have 100R series resistors (R44-R51).

## I2C Expansion (J12)

4-pin connector: 1:GND, 2:3V3, 3:SCL, 4:SDA

## General Expansion (J13)

6-pin connector: 1:GPIO40, 2:GPIO41, 3:3V3, 4:3V3, 5:GND, 6:GND

## VMOT Input (J2)

2-pin connector: 1:VMOT, 2:GND

## USB-C (J1)

USB 2.0 Type-C connector for power and programming.

---

# Power

## Input
- USB-C 5V (5V_USB)
- External VMOT for motors (J2)

## Regulation
- AP63200WU-7 (U3): 5V_USB -> 3V3
- Bulk capacitors: 470uF (C3, C8, C10, C13, C16)

---

# Component Reference

| Designator | Component | Function |
| ---------- | --------- | -------- |
| U1 | ESP32-S3-WROOM-1 | Main MCU |
| U2 | USBLC6-2SC6 | USB ESD protection |
| U3 | AP63200WU-7 | 3.3V regulator |
| U4 | MCP23017-E/SS | GPIO expander |
| U5 | VNH7040AYTR | Motor 1 driver |
| U6 | VNH7040AYTR | Motor 2 driver |
| U7 | VNH7040AYTR | Motor 3 driver |
| U8 | VNH7040AYTR | Motor 4 driver |
| D1 | Red LED | MCP GPA1 controlled |
| D2 | Green LED | MCP GPA2 controlled |
| D5 | Blue LED | ESP32 GPIO42 controlled |
| D6 | Orange LED | Power indicator (3V3) |
| SW1 | Boot button | Enter bootloader |
| SW2 | Reset button | System reset |

---

# Notes

1. Direction control (INA/INB) is via MCP23017, not direct ESP32 GPIO.
2. PWM signals have 1K series resistors (R_PWM1-R_PWM4).
3. INA/INB signals have 1K series resistors (R_INA*, R_INB*).
4. SEL0/SEL1 control MultiSense output mode for all drivers.
5. Encoder signals have 100R series resistors for protection.
6. GPIO40/41 are available on expansion header J13.

---

# Verification

GPIO assignments extracted from KiCad netlist using:
```
kicad-cli sch export netlist -o netlist.net esp32_s3_vnh5019_quad_motor.kicad_sch
```
