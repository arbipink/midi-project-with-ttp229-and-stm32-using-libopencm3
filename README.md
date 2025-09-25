### MIDI Project with libopencm3

This project implements a USB MIDI controller using the STM32F103C8T6 “Blue Pill” board and [libopencm3 midi usb Examples](https://github.com/libopencm3/libopencm3-examples/tree/master/examples/stm32/f4/stm32f4-discovery/usb_midi).

It uses a TTP229 16-key capacitive touch keypad as the input source and sends MIDI messages over USB.

## Hardware Setup

# Hardware Needed
1. MCU Board: STM32F103C8T6 ("Blue Pill")
2. Input Sensor: TTP229 16-key touch sensor (I²C mode)

# Wiring
| TTP229 Pin | STM32F103C8 Pin |
| ---------- | --------------- |
| `SDO`      | `PA6`           |
| `SCL`      | `PA7`           |
| `TP2`      | Connected       |
| `TP3`      | Connected       |
| `TP4`      | Connected       |
| `VCC/GND`  | 3.3V / GND      |

The TP2, TP3, and TP4 pins on the TTP229 side are connected to configure the sensor for 16-key mode and multitouch function.

## Getting Started
# Prerequisites
Make sure you have the following toolchain
```
sudo apt install stlink-tools gcc-arm-none-eabi
```
# Clone the repo
```
git clone --recurse-submodules https://github.com/arbipink/midi-project-with-libopencm3.git
cd midi-project-with-libopencm3
```
# Build
first build libopencm3 (only needed once)
```
make -C libopencm3
```

then build the project
```
make -C my-project
```
# Flash
```
cd my-project
st-flash --reset write midi-project.bin 0x8000000
```

## What i will do in the future
1. Add functionality to change octave
2. Add functionality to control midi cc

## License

- This project’s source code: [MIT License](./LICENSE)  
- libopencm3 dependency: [LGPL v3.0 or later](https://www.gnu.org/licenses/lgpl-3.0.html)  
