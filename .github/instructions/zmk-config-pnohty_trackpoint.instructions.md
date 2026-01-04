# Project Context: ZMK Trackpoint Integration (Sweeq MX)

## Hardware
- **Controller**: Raspberry Pi RP2040-Zero.
- **Trackpoint Module**: IBM/Lenovo Blue Module (SK790902-A2).
- **Wiring**: SCL (Clock) on GP3, SDA (Data) on GP2. Confirmed heartbeat on these pins.
- **Status**: The board previously worked "Laptop Smooth" on a different firmware.

## The Problem
- **Current Driver**: GPIO Bit-Banging (`ps2_gpio.c`).
- **Issues**: Timing jitter caused by CPU interrupts (ZMK BLE/USB stack). 
- **Symptoms**: Random phantom clicks and slow/laggy movement. Packet misalignment occurs because the CPU misses bits during the PS/2 11-bit frame transfer.

## Technical Objective
We need to implement an **RP2040 PIO-based PS/2 driver**. 
1. **PIO Program**: Create a `.pio` file that uses a State Machine to handle the PS/2 protocol asynchronously. It should handle the Start bit, 8 data bits, Parity bit, and Stop bit without main CPU intervention.
2. **Zephyr Driver**: Adapt the existing `ps2_gpio.c` to use the PIO hardware registers instead of `gpio_pin_get_dt`.
3. **Reset Logic**: The Trackpoint often needs a Power-On-Reset (POR). We must ensure the driver can pull SCL/SDA low during init to trigger this.

## Reference Material
- Standard PS/2 Frame: 1 start bit (0), 8 data bits (LSB first), 1 parity bit (odd), 1 stop bit (1).
- RP2040 PIO can emulate UART-like protocols on any GPIO.
- Goal: Move timing-critical logic from the main CPU to the PIO State Machine.

## General instructions
1. **ask vs agent**: If I am just asking you something, do not give me large code blocks. When I ask, I just want to talk about ideas at a high level - I do not want you to give me code to copy and paste. 
2. **commit message**: when I ask you to add, commit, and push code, always give me a detailed commit message that explains what you changed and why.