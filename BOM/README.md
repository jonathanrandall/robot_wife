# Bill of Materials

| Quantity | Component | Notes |
|----------|-----------|-------|
| 8 | **GoBilda 1120 Series U-channel** | Various sizes: 2× 528mm (legs), 2× 432mm (upper body), 2× 360mm (feet), 2× 144mm (crossbars — one between the feet, one between the legs). I have no affiliation with GoBilda and paid full price. |
| 4 | **5204 Series Yellow Jacket Planetary Gear Motor** — 80mm length, 8mm REX shaft, 117 RPM | I'm using the long shaft motor as I had them left over from a previous project. For this build where the motors are orientated at right angles to the wheel axis, short shaft would be better. Four-wheel drive. |
| 100+ | **M4 screws** | You will need lots. For the whole robot I used over 100 M4 screws varying from 6mm to 16mm, plus a number of 60mm screws. Check the GoBilda configuration instructions. |
| 10 | **1201 Series Quadblock Pattern Mount** — 4× (43-5) for motors, 6× (43-2) to connect U-channels | Check the GoBilda website for configuration ideas. |
| 8 | **2319 Series MOD 1.25 Steel Miter Gear** — 8mm REX bore, 30 tooth | Used to mount the motors sideways. You will also need bearings, spacers, and other connection parts — check the GoBilda website for the full list. |
| 1 | **Brackets, bolts, and fixings** | Everything should be connected tightly or the robot will shake while moving. |
| 1 | **Motor driver with ESP32** | I'm using a custom four-motor driver ESP32-S3 PCB that I designed. The KiCad schematic, PCB layout, and bill of materials are available on the [GitHub repo](https://github.com/jonathanrandall/four_motor_controller). |
| 3 | **3-cell LiPo batteries** | 1 for the Raspberry Pi, 2 connected in series for everything else. |
| 1 | **12V regulator** | Needs enough current to drive the motors. My motors have a 9A stall current, so theoretically ~40A is needed, but I think the one I have can handle ~20A — I wouldn't expect to get close to stall current in normal use. |
| 1 | **Pololu 5V 5.5A Step-Down Voltage Regulator D36V50F5** | The only one I found that could drive the Raspberry Pi without crashing. Other 5V regulators worked under normal load but failed when the Pi CPU was running close to its limits. |
| 2 | **Tungkey 5V USB voltage regulators** | USB output. One for the touchscreen, one for everything else needing 5V. |
| 1 | **Variable step-down voltage regulator at 7.5V** | Powers the pan-tilt head servos. |
| 1 | **ESP32 control board for pan-tilt bus servos** | I'm using the Hiwonder pan-tilt with its ESP32 control board. The board has an extra chip for one-wire communication with the bus servos. |
| 1 | **Raspberry Pi 5** | Onboard robot brains. |
| 1 | **USB speaker and microphone** | Connected through a USB/jack converter. |
| 1 | **Waveshare 7" touchscreen** | Input/output touchscreen. |
| 1 | **3D printed parts** | STL files and FreeCAD macros are in the `3d_prints/` directory of this repo. |
| 2 | **Missile switches** | One for the Pi battery, one for the other batteries. |
| 2 | **USB hubs** | One externally powered hub connected to the Pi (should be good quality). One cheap hub for powering various components. |
| 1 | **USB stereo camera** | Connected to the Pi. |
| 1 | **Time-of-flight (ToF) camera** | Arducam ToF, connected via CSI. |
| 4 | **GoBilda Rhino wheels** | 120mm diameter. |
