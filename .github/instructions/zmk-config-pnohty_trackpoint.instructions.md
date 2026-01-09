# Project Context: ZMK Trackpoint Integration (Sweeq MX)

## High-Level Goal
Enable reliable, smooth trackpoint movement on a custom RP2040-Zero based split keyboard using the most efficient driver subsystem available (currently prioritizing PIO-based UART/PS2).

## Hardware Context
* **Controller:** Raspberry Pi RP2040-Zero.
* **PCB:** Sweeq MX (custom designed for split keyboards with trackpoint support).
    - (https://github.com/idank/keyboards/tree/main/sweeq%20mx)
    - "Bling" shield has been working, and I believe that is because that is the name of the Holykeebs hardware revision for the Ferris Sweep that uses the Waveshare RP2040-Zero and supports a Cirque Trackpad
* **Trackpoint:** IBM/Lenovo Blue Module (SK790902-A2).
* **Wiring:** SCL (Clock) on GP3, SDA (Data) on GP2. 
* **Status:** Keyboard matrix works; trackpoint hardware verified to work smoothly on legacy firmware. Have verified functionality on GP2/GP3 with previous builds (cursor movement using trackpoint) - cursor movement was erratic with previous builds, though.

---

## Reference Repositories for Research
The agent should explore these repositories for driver implementations, board definitions, and shield configurations relevant to PS2/UART trackpoints and RP2040-Zero:

* **Primary PS/2 Driver Modules:**
    - [infused-kim/kb_zmk_ps2_mouse_trackpoint_driver](https://github.com/infused-kim/kb_zmk_ps2_mouse_trackpoint_driver): The most widely used external module for PS/2 trackpoints in ZMK. Includes support for sensitivity adjustments and TP-specific commands.
    - [badjeff/kb_zmk_ps2_mouse_trackpoint_driver](https://github.com/badjeff/kb_zmk_ps2_mouse_trackpoint_driver): A fork often updated for compatibility with the latest ZMK mainline/pointing changes.

* **Core Pointer Implementation (Pete Johanson):**
    - [petejohanson/zmk (branch: feat/pointers-move-scroll)](https://github.com/petejohanson/zmk/tree/feat/pointers-move-scroll): Pete's active development branch for the ZMK pointing subsystem. This is the source for the `CONFIG_ZMK_POINTING` features.

* **Hardware & Shield Reference:**
    - [idank/keyboards](https://github.com/idank/keyboards/tree/main/sweeq%20mx): The original hardware repo for the Sweeq MX PCB.

* **Input Processing & Behaviors:**
    - [ZMK Official Documentation - Pointing](https://zmk.dev/docs/features/pointing): Reference for the now-integrated Input Processors (scaling, layers, etc.) that replaced older external listener modules.

---

## General Instructions

### 1. Interaction & Workflow
* **Ask vs Agent:** If I am just asking you something, do not give me large code blocks. Talk about ideas at a high level.
* **Commit Messages:** Always provide a detailed commit message explaining the "what" and "why" of the changes.
* **Terminal Usage:** Whenever possible, execute terminal commands in the existing active terminal session instead of spawning a new background terminal. If a new terminal is required, ensure it sources the `~/.zshrc` profile to maintain visual consistency.

### 2. Environment & Build Setup
* **Build Method:** PRIORITIZE local builds.
* **Environment Check:** Before any build or code change, confirm you are in the `(zmk)` conda environment and on the `left-trackpoint` branch:
    ```bash
    conda activate zmk && git checkout left-trackpoint
    ```

* **Environment Variables - CRITICAL BUILD REQUIREMENT:** You MUST pass environment variables **directly to the west command** using `python3 -m west`. Environment variables set in the shell will NOT be inherited by west's cmake subprocess.
    
    Required variables:
    - `ZEPHYR_BASE`: Absolute path to local zephyr directory
    - `ZEPHYR_SDK_INSTALL_DIR`: Absolute path to zephyr SDK installation (e.g., `/Users/reed/zephyr-sdk-0.16.8` or `$HOME/zephyr-sdk-0.X.X`)
    - `CMAKE_PREFIX_PATH`: Colon-separated paths to: `<SDK>/cmake` and `<ZEPHYR>/share/zephyr-package/cmake`
    
    **Example for Sweeq MX (RP2040-Zero):**
    ```bash
    ZEPHYR_BASE=/Users/reed/dev/zmk-config-pnohty/zephyr \
    ZEPHYR_SDK_INSTALL_DIR=/Users/reed/zephyr-sdk-0.16.8 \
    CMAKE_PREFIX_PATH=/Users/reed/zephyr-sdk-0.16.8/cmake:/Users/reed/dev/zmk-config-pnohty/zephyr/share/zephyr-package/cmake \
    python3 -m west build -b rpi_pico -s zmk/app -- -DSHIELD=sweep_bling_left -DZMK_CONFIG="$PWD/config"
    ```
    
    **For other boards/drivers, substitute:**
    - `<board>`: Board definition (e.g., `rpi_pico`, `rp2040_zero`, `nice_nano`, etc.)
    - `<shield>`: Shield name (e.g., `sweep_bling_left`, `corne_left`, etc.)

* **Build Command Pattern (Generalizable for Multiple Drivers/Boards):**

```bash
ZEPHYR_BASE=<path-to-zephyr> \
ZEPHYR_SDK_INSTALL_DIR=<path-to-sdk> \
CMAKE_PREFIX_PATH=<sdk-path>/cmake:<zephyr-path>/share/zephyr-package/cmake \
python3 -m west build -b <board> -s zmk/app -- -DSHIELD=<shield> -DZMK_CONFIG="$PWD/config"
```

**Note on Board Selection:** RP2040-Zero uses the `rpi_pico` board definition (ZMK standard). A legacy `rp2040_zero` definition exists but relies on slower bit-bang GPIO drivers that introduce jitter. For hardware-timed drivers like Pete Johanson's PIO UART, use `rpi_pico`.

**Boards explored:**
- `rpi_pico` - RP2040 reference board (current)
- `rp2040_zero` - RP2040-Zero variant (explored to a degree)

* **Environment Maintenance:** To save bandwidth, ALWAYS use "skinny" updates. If a `west update` is required, run:
    ```bash
    west update --narrow --fetch-opt=--filter=blob:none --group-filter="+hal_nordic,+hal_rpi_pico,+cmsis,+libmetal,-hal"
    ```
* **Firmware location:** zmk/app/build/zephyr/zmk.uf2

### 3. Git & Staging Protocol
* **NEVER** use `git add .` or `git submodule update`. These will fail due to the "skinny" west setup.
* **ALWAYS** stage files explicitly: 
    ```bash
    git add config/ .github/ *.md
    ```

### 4. Context & Scope
Focus primary attention on these files/folders:
* `config/boards/shields/sweep_bling/`
* `config/west.yml`
* `config/sweep_bling.keymap`
* `config/sweep_bling.conf`
* `.github/workflows/build.yml`

### 5. Logging Protocol
**REFER TO `debug_log.md` BEFORE EVERY TASK.** After every build attempt, update `debug_log.md` following these rules:
* Do not ask me for information you can derive yourself.
* Proactively fill in the **Date**, **Commit Hash**, and **Technical Objective**.
* Mark Hardware Status as **PENDING PHYSICAL VERIFICATION**.
* Never assume a code change fixed the hardware; only document the intent of the change.
* After a build, append the specific Pass/Fail result and any error snippets.
* Identify the next logical step based on findings.