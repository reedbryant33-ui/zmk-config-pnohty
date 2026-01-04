/*
 * Copyright (c) 2023 GitHub Copilot
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT pio_ps2

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/pinctrl.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
// #include <pico/stdlib.h> // REMOVED: Use Zephyr APIs instead

#include "ps2_rx.pio.h" // Generated PIO header

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_pio);

#define PS2_PIO_DATA_QUEUE_SIZE 100

static PIO pio = pio0; // Use PIO0
static uint sm = 0;    // Use state machine 0

struct ps2_pio_data {
    const struct device *dev;
    ps2_callback_t callback_isr;
    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;
    uint8_t data_queue_buffer[PS2_PIO_DATA_QUEUE_SIZE];
    struct k_msgq data_queue;
    struct k_work callback_work; // For scheduling the callback
    uint8_t callback_byte;       // Byte to pass to the callback
};

struct ps2_pio_config {
    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;
    const struct pinctrl_dev_config *pcfg;
};

void ps2_pio_read_callback_work_handler(struct k_work *work) {
    struct ps2_pio_data *data = CONTAINER_OF(work, struct ps2_pio_data, callback_work);
    if (data->callback_isr) {
        data->callback_isr(data->dev, data->callback_byte);
    }
}

static void ps2_pio_isr(void *arg) {
    const struct device *dev = (const struct device *)arg;
    struct ps2_pio_data *data = dev->data;

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint32_t raw_frame = pio_sm_get(pio, sm);

        // The PIO program pushes 11 bits: [stop_bit][parity_bit][data_7]...[data_0][start_bit]
        // (assuming in_shiftdir is right, so first bit read goes to bit 0)
        uint8_t start_bit = (raw_frame >> 0) & 0x1;
        uint8_t data_byte = (raw_frame >> 1) & 0xFF;
        uint8_t parity_bit = (raw_frame >> 9) & 0x1;
        uint8_t stop_bit = (raw_frame >> 10) & 0x1;

        // Basic validation
        if (start_bit != 0) {
            LOG_WRN("PS/2 PIO: Invalid start bit (0x%x)", start_bit);
            continue;
        }
        if (stop_bit != 1) {
            LOG_WRN("PS/2 PIO: Invalid stop bit (0x%x)", stop_bit);
            continue;
        }
        // Odd parity check: __builtin_parity returns 1 for odd number of set bits.
        // PS/2 uses odd parity, meaning the total number of set bits (data + parity) must be odd.
        // So, if data_byte has an even number of set bits, parity_bit must be 1.
        // If data_byte has an odd number of set bits, parity_bit must be 0.
        bool calculated_parity = !__builtin_parity(data_byte); // 1 if data_byte has even set bits, 0 if odd
        if (calculated_parity != parity_bit) {
            LOG_WRN("PS/2 PIO: Parity error for byte 0x%x (expected %d, got %d)", data_byte, calculated_parity, parity_bit);
            // Optionally, call resend_callback_isr if enabled
            continue;
        }

        // Add to data queue for ps2_pio_read
        if (k_msgq_put(&data->data_queue, &data_byte, K_NO_WAIT) != 0) {
            uint8_t dropped_byte;
            k_msgq_get(&data->data_queue, &dropped_byte, K_NO_WAIT); // Drop oldest
            k_msgq_put(&data->data_queue, &data_byte, K_NO_WAIT);
        }

        // Schedule work to call callback_isr
        if (data->callback_isr) {
            data->callback_byte = data_byte;
            k_work_submit(&data->callback_work);
        }
    }
    pio_interrupt_clear(pio, PIO_IRQ0_INTE_SM0_RXNEMPTY_LSB << sm);
}

static int ps2_pio_read(const struct device *dev, uint8_t *value) {
    struct ps2_pio_data *data = dev->data;
    if (k_msgq_get(&data->data_queue, value, K_FOREVER) != 0) { // K_FOREVER or a timeout
        return -ETIMEDOUT; // Or other error
    }
    return 0;
}

static int ps2_pio_write(const struct device *dev, uint8_t value) {
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;

    // Temporarily disable PIO RX and configure pins as GPIO outputs for writing
    pio_sm_set_enabled(pio, sm, false);

    gpio_pin_configure_dt(&config->scl_gpio, GPIO_OUTPUT);
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_OUTPUT);

    // PS/2 Host-to-Device Transmission Sequence:
    // 1. Host pulls Clock low for at least 100us (inhibit).
    gpio_pin_set_dt(&config->scl_gpio, 0);
    k_busy_wait(120); // Inhibit time

    // 2. Host pulls Data low.
    gpio_pin_set_dt(&config->sda_gpio, 0);
    k_busy_wait(10); // Small delay

    // 3. Host releases Clock (Clock goes high, driven by pull-up).
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_INPUT | GPIO_PULL_UP); // Release SCL
    k_busy_wait(10); // Wait for SCL to go high

    // Wait for device to take over SCL (first falling edge)
    int timeout_us = 5000; // Max 5ms for device to start clocking
    while (gpio_pin_get_dt(&config->scl_gpio) == 1 && timeout_us > 0) {
        k_busy_wait(1);
        timeout_us--;
    }
    if (timeout_us <= 0) {
        LOG_ERR("PS/2 PIO: Write timeout waiting for SCL low");
        goto re_enable_pio;
    }

    // Construct 11-bit frame: [stop_bit][parity_bit][data_7]...[data_0][start_bit]
    uint16_t frame = 0;
    frame |= (value & 0xFF) << 1; // Data bits (LSB first)
    frame |= (0 & 0x1) << 0;      // Start bit (always 0)
    frame |= (1 & 0x1) << 10;     // Stop bit (always 1)
    bool calculated_parity = !__builtin_parity(value); // 1 if even set bits, 0 if odd
    frame |= (calculated_parity & 0x1) << 9; // Parity bit

    for (int i = 0; i < 11; i++) {
        // Wait for SCL low (device clock)
        timeout_us = 1000; // Max 1ms
        while (gpio_pin_get_dt(&config->scl_gpio) == 1 && timeout_us > 0) {
            k_busy_wait(1);
            timeout_us--;
        }
        if (timeout_us <= 0) {
            LOG_ERR("PS/2 PIO: Write timeout waiting for SCL low (bit %d)", i);
            goto re_enable_pio;
        }

        // Set SDA
        gpio_pin_set_dt(&config->sda_gpio, (frame >> i) & 0x1);

        // Wait for SCL high (device clock)
        timeout_us = 1000; // Max 1ms
        while (gpio_pin_get_dt(&config->scl_gpio) == 0 && timeout_us > 0) {
            k_busy_wait(1);
            timeout_us--;
        }
        if (timeout_us <= 0) {
            LOG_ERR("PS/2 PIO: Write timeout waiting for SCL high (bit %d)", i);
            goto re_enable_pio;
        }
    }

    // After sending 11 bits, release SDA (let pull-up make it high)
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT | GPIO_PULL_UP);

    // Wait for device ACK (SDA low for one clock period)
    timeout_us = 1000; // Max 1ms
    while (gpio_pin_get_dt(&config->sda_gpio) == 1 && timeout_us > 0) { // Wait for SDA low (ACK)
        k_busy_wait(1);
        timeout_us--;
    }
    if (timeout_us <= 0) {
        LOG_ERR("PS/2 PIO: Write timeout waiting for ACK");
        goto re_enable_pio;
    }

    // Wait for device to release SDA (SDA high)
    timeout_us = 1000; // Max 1ms
    while (gpio_pin_get_dt(&config->sda_gpio) == 0 && timeout_us > 0) { // Wait for SDA high
        k_busy_wait(1);
        timeout_us--;
    }
    if (timeout_us <= 0) {
        LOG_ERR("PS/2 PIO: Write timeout waiting for SDA release");
        goto re_enable_pio;
    }

re_enable_pio:
    // Re-enable PIO read by setting pins back to PIO function
    pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    pio_sm_set_enabled(pio, sm, true);

    return (timeout_us > 0) ? 0 : -ETIMEDOUT;
}

static int ps2_pio_configure(const struct device *dev, ps2_callback_t callback_isr) {
    struct ps2_pio_data *data = dev->data;
    data->callback_isr = callback_isr;
    return 0;
}

static int ps2_pio_disable_callback(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    data->callback_isr = NULL; // Simply clear the callback
    return 0;
}

static int ps2_pio_enable_callback(const struct device *dev) {
    // The PIO is always enabled for reading. The callback is set via ps2_pio_configure.
    // This function doesn't have a way to restore it if it was set to NULL.
    return 0;
}

static const struct ps2_driver_api ps2_pio_driver_api = {
    .config = ps2_pio_configure,
    .read = ps2_pio_read,
    .write = ps2_pio_write,
    .disable_callback = ps2_pio_disable_callback,
    .enable_callback = ps2_pio_enable_callback,
};

static int ps2_pio_init(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;

    data->dev = dev;
    data->scl_gpio = config->scl_gpio;
    data->sda_gpio = config->sda_gpio;

    k_msgq_init(&data->data_queue, data->data_queue_buffer, sizeof(uint8_t), PS2_PIO_DATA_QUEUE_SIZE);
    k_work_init(&data->callback_work, ps2_pio_read_callback_work_handler);

    LOG_INF("PS/2 PIO Initializing on SCL GP%d, SDA GP%d", config->scl_gpio.pin, config->sda_gpio.pin);

    // --- Power-On-Reset (POR) handling: Pull SCL/SDA low during initialization ---
    LOG_INF("PS/2 PIO: Performing SCL/SDA Power-On-Reset...");
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_OUTPUT_INACTIVE);
    k_sleep(K_MSEC(100)); // Hold low for 100ms

    // Release SCL and SDA (let pull-ups take over) by setting them as inputs
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT | GPIO_PULL_UP);
    k_sleep(K_MSEC(100)); // Wait for lines to stabilize high
    // --- End of POR handling ---

    // PIO initialization
    if (pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
        LOG_ERR("Failed to apply pinctrl state");
        return -EIO;
    }

    uint offset = pio_add_program(pio, &ps2_rx_program);
    sm = pio_claim_unused_sm(pio, true); // Claim an unused state machine

    pio_sm_config c = ps2_rx_program_get_default_config(offset);

    // Set 'in' pins base to SDA
    sm_config_set_in_pins(&c, config->sda_gpio.pin);
    // Set 'jmp' pin to SCL for `wait` instructions
    sm_config_set_jmp_pin(&c, config->scl_gpio.pin);

    // Shift right, autopush 11 bits (start, 8 data, parity, stop)
    sm_config_set_in_shift(&c, true, true, 11);

    pio_sm_init(pio, sm, offset, &c);

    // PIO pin configuration is now handled by pinctrl in the devicetree

    // Enable PIO interrupt for RX FIFO not empty
    pio_set_irq0_enabled(pio, PIO_IRQ0_INTE_SM0_RXNEMPTY_LSB << sm, true);
    IRQ_CONNECT(PIO0_IRQ_0, 0, ps2_pio_isr, DEVICE_DT_INST_GET(0), 0);
    irq_enable(PIO0_IRQ_0);

    pio_sm_set_enabled(pio, sm, true);

    return 0;
}

// Device tree instantiation
#define PS2_PIO_DEFINE(n)                                                                          \
    PINCTRL_DT_INST_DEFINE(n);                                                                     \
    static struct ps2_pio_data data##n;                                                            \
    static const struct ps2_pio_config config##n = {                                               \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                                           \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                                           \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &ps2_pio_init, NULL, &data##n, &config##n,                            \
                          POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,                                   \
                          &ps2_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_PIO_DEFINE)
