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

#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <pico/stdlib.h> // For gpio_set_function, etc.

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
};

void ps2_pio_read_callback_work_handler(struct k_work *work) {
    struct ps2_pio_data *data = CONTAINER_OF(work, struct ps2_pio_data, callback_work);
    if (data->callback_isr) {
        data->callback_isr(data->dev, data->callback_byte);
    }
}

static void ps2_pio_isr(PIO pio, uint sm) {
    struct ps2_pio_data *data = (struct ps2_pio_data *)pio_sm_get_user_data(pio, sm);
    while (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
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
            k_msgq_get(&data->data_queue, &data_byte, K_NO_WAIT); // Drop oldest
            k_msgq_put(&data->data_queue, &data_byte, K_NO_WAIT);
        }

        // Schedule work to call callback_isr
        if (data->callback_isr) {
            data->callback_byte = data_byte;
            k_work_submit(&data->callback_work);
        }
    }
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
    pio_set_input_sync_bypass(pio, sm, false); // Disable bypass for PIO to control pins

    gpio_set_function(config->scl_gpio.pin, GPIO_FUNC_SIO);
    gpio_set_dir(config->scl_gpio.pin, GPIO_OUT);
    gpio_set_function(config->sda_gpio.pin, GPIO_FUNC_SIO);
    gpio_set_dir(config->sda_gpio.pin, GPIO_OUT);

    // PS/2 Host-to-Device Transmission Sequence:
    // 1. Host pulls Clock low for at least 100us (inhibit).
    gpio_put(config->scl_gpio.pin, 0);
    k_sleep(K_USEC(120)); // Inhibit time

    // 2. Host pulls Data low.
    gpio_put(config->sda_gpio.pin, 0);
    k_sleep(K_USEC(10)); // Small delay

    // 3. Host releases Clock (Clock goes high, driven by pull-up).
    gpio_put(config->scl_gpio.pin, 1); // Set high, then configure as input
    gpio_set_dir(config->scl_gpio.pin, GPIO_IN); // Release SCL
    k_sleep(K_USEC(10)); // Wait for SCL to go high

    // Wait for device to take over SCL (first falling edge)
    int timeout_us = 5000; // Max 5ms for device to start clocking
    int scl_val = gpio_get(config->scl_gpio.pin);
    while (scl_val == 1 && timeout_us > 0) {
        k_sleep(K_USEC(1));
        scl_val = gpio_get(config->scl_gpio.pin);
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
        scl_val = gpio_get(config->scl_gpio.pin);
        while (scl_val == 1 && timeout_us > 0) {
            k_sleep(K_USEC(1));
            scl_val = gpio_get(config->scl_gpio.pin);
            timeout_us--;
        }
        if (timeout_us <= 0) {
            LOG_ERR("PS/2 PIO: Write timeout waiting for SCL low (bit %d)", i);
            goto re_enable_pio;
        }

        // Set SDA
        gpio_put(config->sda_gpio.pin, (frame >> i) & 0x1);

        // Wait for SCL high (device clock)
        timeout_us = 1000; // Max 1ms
        scl_val = gpio_get(config->scl_gpio.pin);
        while (scl_val == 0 && timeout_us > 0) {
            k_sleep(K_USEC(1));
            scl_val = gpio_get(config->scl_gpio.pin);
            timeout_us--;
        }
        if (timeout_us <= 0) {
            LOG_ERR("PS/2 PIO: Write timeout waiting for SCL high (bit %d)", i);
            goto re_enable_pio;
        }
    }

    // After sending 11 bits, release SDA (let pull-up make it high)
    gpio_put(config->sda_gpio.pin, 1);
    gpio_set_dir(config->sda_gpio.pin, GPIO_IN);

    // Wait for device ACK (SDA low for one clock period)
    timeout_us = 1000; // Max 1ms
    int sda_val = gpio_get(config->sda_gpio.pin);
    while (sda_val == 1 && timeout_us > 0) { // Wait for SDA low (ACK)
        k_sleep(K_USEC(1));
        sda_val = gpio_get(config->sda_gpio.pin);
        timeout_us--;
    }
    if (timeout_us <= 0) {
        LOG_ERR("PS/2 PIO: Write timeout waiting for ACK");
        goto re_enable_pio;
    }

    // Wait for device to release SDA (SDA high)
    timeout_us = 1000; // Max 1ms
    sda_val = gpio_get(config->sda_gpio.pin);
    while (sda_val == 0 && timeout_us > 0) { // Wait for SDA high
        k_sleep(K_USEC(1));
        sda_val = gpio_get(config->sda_gpio.pin);
        timeout_us--;
    }
    if (timeout_us <= 0) {
        LOG_ERR("PS/2 PIO: Write timeout waiting for SDA release");
        goto re_enable_pio;
    }

re_enable_pio:
    // Re-enable PIO read
    pio_gpio_init(pio, config->scl_gpio.pin);
    pio_gpio_init(pio, config->sda_gpio.pin);
    gpio_set_function(config->scl_gpio.pin, GPIO_FUNC_PIO0);
    gpio_set_function(config->sda_gpio.pin, GPIO_FUNC_PIO0);
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
    // Temporarily configure SCL/SDA as GPIO outputs for POR sequence
    gpio_init(config->scl_gpio.pin);
    gpio_set_dir(config->scl_gpio.pin, GPIO_OUT);
    gpio_init(config->sda_gpio.pin);
    gpio_set_dir(config->sda_gpio.pin, GPIO_OUT);

    LOG_INF("PS/2 PIO: Performing SCL/SDA Power-On-Reset...");
    gpio_put(config->scl_gpio.pin, 0);
    gpio_put(config->sda_gpio.pin, 0);
    k_sleep(K_MSEC(100)); // Hold low for 100ms

    // Release SCL and SDA (let pull-ups take over)
    gpio_put(config->scl_gpio.pin, 1);
    gpio_put(config->sda_gpio.pin, 1);
    k_sleep(K_MSEC(100)); // Wait for lines to stabilize high
    // --- End of POR handling ---

    // PIO initialization
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

    // Configure SCL and SDA as PIO controlled inputs with pull-ups
    pio_gpio_init(pio, config->scl_gpio.pin);
    pio_gpio_init(pio, config->sda_gpio.pin);
    gpio_set_pulls(config->scl_gpio.pin, true, false); // SCL pull-up
    gpio_set_pulls(config->sda_gpio.pin, true, false); // SDA pull-up
    // Ensure input override is normal for PIO to control
    gpio_set_input_override(config->scl_gpio.pin, GPIO_OVERRIDE_NORMAL);
    gpio_set_input_override(config->sda_gpio.pin, GPIO_OVERRIDE_NORMAL);

    // Set PIO pins as inputs
    pio_sm_set_consecutive_pindirs(pio, sm, config->scl_gpio.pin, 1, false); // SCL as input
    pio_sm_set_consecutive_pindirs(pio, sm, config->sda_gpio.pin, 1, false); // SDA as input

    // Enable PIO interrupt for RX FIFO not empty
    pio_set_irq0_enabled(pio, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, ps2_pio_isr);
    irq_set_priority(PIO0_IRQ_0, 0); // Highest priority for PIO ISR
    irq_set_enabled(PIO0_IRQ_0, true);

    pio_sm_set_user_data(pio, sm, data); // Store data pointer for ISR

    pio_sm_set_enabled(pio, sm, true);

    return 0;
}

// Device tree instantiation
#define PS2_PIO_DEFINE(n)                                                                          \
    static struct ps2_pio_data data##n;                                                            \
    static const struct ps2_pio_config config##n = {                                               \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                                           \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &ps2_pio_init, NULL, &data##n, &config##n,                            \
                          POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,                                   \
                          &ps2_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_PIO_DEFINE)
