# Project Context: ZMK Trackpoint Integration (Sweeq MX)

## High-Level Goal
Enable reliable, smooth trackpoint movement on a custom RP2040-Zero based split keyboard using the most efficient driver subsystem available (currently prioritizing PIO-based UART/PS2).

## Hardware Context
- **Controller**: Raspberry Pi RP2040-Zero.
- **Trackpoint**: IBM/Lenovo Blue Module (SK790902-A2).
- **Wiring**: SCL (Clock) on GP3, SDA (Data) on GP2. 
- **Status**: Keyboard matrix works; trackpoint hardware verified on legacy firmware.

## General Instructions
1. **ask vs agent**: If I am just asking you something, do not give me large code blocks. Talk about ideas at a high level.
2. **commit message**: Always provide a detailed commit message explaining the "what" and "why" of the changes.
3. **build method**: ALWAYS use GitHub Actions for builds. Do not attempt local 'west build'. 
4. **context & scope**: Focus primary attention on these files/folders:
    - `config/boards/shields/sweep_bling/`
    - `config/west.yml`
    - `config/sweep_bling.keymap`
    - `config/sweep_bling.conf`
    - `.github/workflows/build.yml`
5. **logging protocol**: REFER TO `debug_log.md` BEFORE EVERY TASK. After every build attempt, update `debug_log.md` with:
    - Date/Time & Commit Hash.
    - Result (Pass/Fail) and specific error logs if failed.
    - Current state of hardware (e.g., "Keys work, no pointer movement").
    - Next logical step based on findings.