# Project Context: ZMK Trackpoint Integration (Sweeq MX)

## High-Level Goal
Enable reliable, smooth trackpoint movement on a custom RP2040-Zero based split keyboard using the most efficient driver subsystem available (currently prioritizing PIO-based UART/PS2).

## Hardware Context
* **Controller:** Raspberry Pi RP2040-Zero.
* **Trackpoint:** IBM/Lenovo Blue Module (SK790902-A2).
* **Wiring:** SCL (Clock) on GP3, SDA (Data) on GP2. 
* **Status:** Keyboard matrix works; trackpoint hardware verified on legacy firmware.

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
* **Environment Variables:** You MUST export the following before building:
    ```bash
    export ZEPHYR_BASE=$HOME/dev/zmk-config-pnohty/zephyr
    export ZEPHYR_SDK_INSTALL_DIR=$HOME/zephyr-sdk-0.16.8
    export CMAKE_PREFIX_PATH=$ZEPHYR_BASE
    ```
* **Build Command:** Use the specific `west build` command for `rpi_pico` and the `sweep_bling` shield.
* **Environment Maintenance:** To save bandwidth on the 11 Mbps connection, ALWAYS use "skinny" updates. If a `west update` is required, run:
    ```bash
    west update --narrow --fetch-opt=--filter=blob:none --group-filter="+hal_nordic,+hal_rpi_pico,+cmsis,+libmetal,-hal"
    ```

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