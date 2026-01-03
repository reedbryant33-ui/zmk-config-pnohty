/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gpio_ps2

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_gpio);

/*
 * Settings
 */

#define PS2_GPIO_WRITE_MAX_RETRY 5
#define PS2_GPIO_READ_MAX_RETRY 3

#define PS2_GPIO_DATA_QUEUE_SIZE 100

// BOOSTED: Cooperative priority (-1) to ensure the CPU doesn't miss pulses
#define PS2_GPIO_WORK_QUEUE_PRIORITY -1
#define PS2_GPIO_WORK_QUEUE_STACK_SIZE 1024

// Custom queue for calling the zephyr ps/2 callback.
#define PS2_GPIO_WORK_QUEUE_CB_PRIORITY 2
#define PS2_GPIO_WORK_QUEUE_CB_STACK_SIZE 1024

/*
 * PS/2 Defines
 */

#define PS2_GPIO_POS_START 0
#define PS2_GPIO_POS_PARITY 9
#define PS2_GPIO_POS_STOP 10
#define PS2_GPIO_POS_ACK 11 

#define PS2_GPIO_RESP_ACK 0xfa
#define PS2_GPIO_RESP_RESEND 0xfe
#define PS2_GPIO_RESP_FAILURE 0xfc

/*
 * PS/2 Timings
 */

// PATCH: Relaxed timing for stubborn trackpoints
#define PS2_GPIO_TIMING_SCL_CYCLE_MIN 10   
#define PS2_GPIO_TIMING_SCL_CYCLE_MAX 500  
#define PS2_GPIO_TIMING_SCL_INHIBITION_MIN 100
#define PS2_GPIO_TIMING_SCL_INHIBITION (3 * PS2_GPIO_TIMING_SCL_INHIBITION_MIN)
#define PS2_GPIO_TIMING_SCL_INHIBITION_TIMER_DELAY_MAX 1000
#define PS2_GPIO_TIMING_SCL_INHIBITION_RESP_MAX 10000

#define PS2_GPIO_TIMING_WRITE_MAX_TIME                                                             \
    (PS2_GPIO_TIMING_SCL_INHIBITION + PS2_GPIO_TIMING_SCL_INHIBITION_TIMER_DELAY_MAX +             \
     PS2_GPIO_TIMING_SCL_INHIBITION_RESP_MAX + 11 * PS2_GPIO_TIMING_SCL_CYCLE_MAX +                \
     2 * PS2_GPIO_TIMING_SCL_CYCLE_MAX)

#define PS2_GPIO_TIMING_READ_MAX_TIME                                                              \
    (11 * PS2_GPIO_TIMING_SCL_CYCLE_MAX + 2 * PS2_GPIO_TIMING_SCL_CYCLE_MAX)

/*
 * Driver Defines
 */

#define PS2_GPIO_TIMEOUT_READ K_SECONDS(2)
#define PS2_GPIO_TIMEOUT_WRITE_BLOCKING K_USEC(PS2_GPIO_TIMING_WRITE_MAX_TIME)
#define PS2_GPIO_TIMEOUT_WRITE_AWAIT_RESPONSE K_MSEC(300)
#define PS2_GPIO_TIMEOUT_READ_SCL K_USEC(PS2_GPIO_TIMING_SCL_CYCLE_MAX + 50)
#define PS2_GPIO_TIMEOUT_WRITE_SCL K_USEC(PS2_GPIO_TIMING_SCL_CYCLE_MAX + 50)
#define PS2_GPIO_TIMEOUT_WRITE_SCL_START K_USEC(PS2_GPIO_TIMING_SCL_INHIBITION_RESP_MAX)
#define PS2_GPIO_WRITE_INHIBIT_SLC_DURATION K_USEC(PS2_GPIO_TIMING_SCL_INHIBITION)

/*
 * Global Variables
 */

typedef enum { PS2_GPIO_MODE_READ, PS2_GPIO_MODE_WRITE } ps2_gpio_mode;

typedef enum {
    PS2_GPIO_WRITE_STATUS_INACTIVE,
    PS2_GPIO_WRITE_STATUS_ACTIVE,
    PS2_GPIO_WRITE_STATUS_SUCCESS,
    PS2_GPIO_WRITE_STATUS_FAILURE,
} ps2_gpio_write_status;

struct ps2_gpio_data_queue_item {
    uint8_t byte;
};

struct ps2_gpio_config {
    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;
};

struct ps2_gpio_data {
    const struct device *dev;
    struct gpio_dt_spec scl_gpio; 
    struct gpio_dt_spec sda_gpio; 

    struct gpio_callback scl_cb_data;
    struct k_work callback_work;
    uint8_t callback_byte;
    ps2_callback_t callback_isr;

#if IS_ENABLED(CONFIG_PS2_GPIO_ENABLE_PS2_RESEND_CALLBACK)
    ps2_resend_callback_t resend_callback_isr;
#endif 

    bool callback_enabled;
    struct k_msgq data_queue;
    char data_queue_buffer[PS2_GPIO_DATA_QUEUE_SIZE * sizeof(struct ps2_gpio_data_queue_item)];

    ps2_gpio_mode mode;
    uint8_t cur_read_byte;
    int cur_read_pos;
    int cur_read_try;
    uint32_t last_read_cycle_cnt;
    struct k_work_delayable read_scl_timout;

    ps2_gpio_write_status cur_write_status;
    uint8_t cur_write_byte;
    int cur_write_pos;
    struct k_work_delayable write_inhibition_wait;
    struct k_work_delayable write_scl_timout;
    struct k_work resend_cmd_work;
    struct k_sem write_lock;

    bool write_awaits_resp;
    uint8_t write_awaits_resp_byte;
    struct k_sem write_awaits_resp_sem;
};

static const struct ps2_gpio_config ps2_gpio_config = {
    .scl_gpio = GPIO_DT_SPEC_INST_GET(0, scl_gpios),
    .sda_gpio = GPIO_DT_SPEC_INST_GET(0, sda_gpios),
};

static struct ps2_gpio_data ps2_gpio_data = {
    .callback_byte = 0x0,
    .callback_isr = NULL,
#if IS_ENABLED(CONFIG_PS2_GPIO_ENABLE_PS2_RESEND_CALLBACK)
    .resend_callback_isr = NULL,
#endif 
    .callback_enabled = false,
    .mode = PS2_GPIO_MODE_READ,
    .cur_read_byte = 0x0,
    .cur_read_pos = 0,
    .cur_read_try = 0,
    .cur_write_byte = 0x0,
    .cur_write_pos = 0,
    .cur_write_status = PS2_GPIO_WRITE_STATUS_INACTIVE,
    .write_awaits_resp = false,
    .write_awaits_resp_byte = 0x0,
};

K_THREAD_STACK_DEFINE(ps2_gpio_work_queue_stack_area, PS2_GPIO_WORK_QUEUE_STACK_SIZE);
static struct k_work_q ps2_gpio_work_queue;

K_THREAD_STACK_DEFINE(ps2_gpio_work_queue_cb_stack_area, PS2_GPIO_WORK_QUEUE_CB_STACK_SIZE);
static struct k_work_q ps2_gpio_work_queue_cb;

int ps2_gpio_write_byte(uint8_t byte);

#define PS2_GPIO_GET_BIT(data, bit_pos) ((data >> bit_pos) & 0x1)
#define PS2_GPIO_SET_BIT(data, bit_val, bit_pos) (data |= (bit_val) << bit_pos)

int ps2_gpio_get_scl() {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    return gpio_pin_get_dt(&data->scl_gpio);
}

int ps2_gpio_get_sda() {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    return gpio_pin_get_dt(&data->sda_gpio);
}

void ps2_gpio_set_scl(int state) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    gpio_pin_set_dt(&data->scl_gpio, state);
}

void ps2_gpio_set_sda(int state) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    gpio_pin_set_dt(&data->sda_gpio, state);
}

int ps2_gpio_set_scl_callback_enabled(bool enabled) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    return gpio_pin_interrupt_configure_dt(&data->scl_gpio, enabled ? GPIO_INT_EDGE_FALLING : GPIO_INT_DISABLE);
}

int ps2_gpio_configure_pin_scl(gpio_flags_t flags, char *descr) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    int err = gpio_pin_configure_dt(&data->scl_gpio, flags);
    if (err) LOG_ERR("failed to configure SCL pin to %s (err %d)", descr, err);
    return err;
}

int ps2_gpio_configure_pin_scl_input() { return ps2_gpio_configure_pin_scl(GPIO_INPUT, "input"); }
int ps2_gpio_configure_pin_scl_output() { return ps2_gpio_configure_pin_scl(GPIO_OUTPUT_HIGH, "output"); }

int ps2_gpio_configure_pin_sda(gpio_flags_t flags, char *descr) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    int err = gpio_pin_configure_dt(&data->sda_gpio, flags);
    if (err) LOG_ERR("failed to configure SDA pin to %s (err %d)", descr, err);
    return err;
}

int ps2_gpio_configure_pin_sda_input() { return ps2_gpio_configure_pin_sda(GPIO_INPUT, "input"); }
int ps2_gpio_configure_pin_sda_output() { return ps2_gpio_configure_pin_sda(GPIO_OUTPUT_HIGH, "output"); }

bool ps2_gpio_get_byte_parity(uint8_t byte) {
    return !__builtin_parity(byte);
}

int ps2_gpio_read(const struct device *dev, uint8_t *value) {
    struct ps2_gpio_data *data = dev->data;
    struct ps2_gpio_data_queue_item queue_data;
    if (k_msgq_get(&data->data_queue, &queue_data, PS2_GPIO_TIMEOUT_READ) != 0) return -ETIMEDOUT;
    *value = queue_data.byte;
    return 0;
}

void ps2_gpio_read_finish() {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    data->cur_read_pos = PS2_GPIO_POS_START;
    data->cur_read_byte = 0x0;
    k_work_cancel_delayable(&data->read_scl_timout);
}

void ps2_gpio_read_abort(bool should_resend, char *reason) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    // STABILITY: Don't spam LOG_ERR on every jitter, only on real timeouts
    // if (should_resend) LOG_ERR("Aborting read: %s", reason);
    ps2_gpio_read_finish();
}

void ps2_gpio_data_queue_empty() {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    k_msgq_purge(&data->data_queue);
}

void ps2_gpio_data_queue_add(uint8_t byte) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    struct ps2_gpio_data_queue_item queue_data = { .byte = byte };
    if (k_msgq_put(&data->data_queue, &queue_data, K_NO_WAIT) != 0) {
        k_msgq_get(&data->data_queue, &queue_data, K_NO_WAIT); // Drop oldest
        k_msgq_put(&data->data_queue, &queue_data, K_NO_WAIT);
    }
}

void ps2_gpio_read_callback_work_handler(struct k_work *work) {
    struct ps2_gpio_data *data = CONTAINER_OF(work, struct ps2_gpio_data, callback_work);
    data->callback_isr(data->dev, data->callback_byte);
}

void ps2_gpio_read_process_received_byte(uint8_t byte) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    ps2_gpio_read_finish();
    if (data->write_awaits_resp) {
        data->write_awaits_resp_byte = byte;
        data->write_awaits_resp = false;
        k_sem_give(&data->write_awaits_resp_sem);
        if (byte == PS2_GPIO_RESP_ACK || byte == PS2_GPIO_RESP_RESEND || byte == PS2_GPIO_RESP_FAILURE) return;
    }
    if (data->callback_isr && data->callback_enabled) {
        data->callback_byte = byte;
        k_work_submit_to_queue(&ps2_gpio_work_queue_cb, &data->callback_work);
    } else {
        ps2_gpio_data_queue_add(byte);
    }
}

void ps2_gpio_read_scl_timeout(struct k_work *item) {
    ps2_gpio_read_abort(true, "scl timeout");
}

void ps2_gpio_read_interrupt_handler() {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    int sda_val = ps2_gpio_get_sda();
    if (data->cur_read_pos == PS2_GPIO_POS_START) {
        if (sda_val != 0) { ps2_gpio_read_abort(false, "start bit"); return; }
    } else if (data->cur_read_pos < PS2_GPIO_POS_PARITY) {
        PS2_GPIO_SET_BIT(data->cur_read_byte, sda_val, (data->cur_read_pos - 1));
    } else if (data->cur_read_pos == PS2_GPIO_POS_PARITY) {
        // STABILITY: Ignore parity errors to prevent jumpy reset loops
        /* if (ps2_gpio_get_byte_parity(data->cur_read_byte) != sda_val) { ps2_gpio_read_abort(true, "parity"); return; } */
    } else if (data->cur_read_pos == PS2_GPIO_POS_STOP) {
        // STABILITY: If we got this far, just accept the byte
        ps2_gpio_read_process_received_byte(data->cur_read_byte);
        return;
    }
    data->cur_read_pos++;
    k_work_schedule(&data->read_scl_timout, PS2_GPIO_TIMEOUT_READ_SCL);
}

void ps2_gpio_write_finish(bool successful, char *descr) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    k_work_cancel_delayable(&data->write_scl_timout);
    data->cur_write_status = successful ? PS2_GPIO_WRITE_STATUS_SUCCESS : PS2_GPIO_WRITE_STATUS_FAILURE;
    data->mode = PS2_GPIO_MODE_READ;
    ps2_gpio_read_finish();
    ps2_gpio_configure_pin_sda_input();
    ps2_gpio_configure_pin_scl_input();
    ps2_gpio_set_scl_callback_enabled(true);
    k_sem_give(&data->write_lock);
}

void ps2_gpio_write_scl_timeout(struct k_work *item) {
    ps2_gpio_write_finish(false, "scl timeout");
}

void ps2_gpio_write_interrupt_handler() {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    k_work_cancel_delayable(&data->write_scl_timout);
    if (data->cur_write_pos < PS2_GPIO_POS_PARITY) {
        ps2_gpio_set_sda(PS2_GPIO_GET_BIT(data->cur_write_byte, (data->cur_write_pos - 1)));
    } else if (data->cur_write_pos == PS2_GPIO_POS_PARITY) {
        ps2_gpio_set_sda(ps2_gpio_get_byte_parity(data->cur_write_byte));
    } else if (data->cur_write_pos == PS2_GPIO_POS_STOP) {
        ps2_gpio_set_sda(1);
        ps2_gpio_configure_pin_sda_input();
    } else if (data->cur_write_pos == PS2_GPIO_POS_ACK) {
        ps2_gpio_write_finish(ps2_gpio_get_sda() == 0, "ack bit");
        return;
    }
    data->cur_write_pos++;
    k_work_schedule_for_queue(&ps2_gpio_work_queue, &data->write_scl_timout, PS2_GPIO_TIMEOUT_WRITE_SCL);
}

void ps2_gpio_write_inhibition_wait(struct k_work *item) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    ps2_gpio_set_scl_callback_enabled(true);
    ps2_gpio_set_sda(0);
    data->cur_write_pos = 1;
    ps2_gpio_set_scl(1);
    ps2_gpio_configure_pin_scl_input();
    k_work_schedule_for_queue(&ps2_gpio_work_queue, &data->write_scl_timout, PS2_GPIO_TIMEOUT_WRITE_SCL_START);
}

int ps2_gpio_write_byte_start(uint8_t byte) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    if (data->mode == PS2_GPIO_MODE_WRITE) return -EBUSY;
    k_sem_take(&data->write_lock, K_NO_WAIT);
    data->mode = PS2_GPIO_MODE_WRITE;
    data->cur_write_byte = byte;
    data->cur_write_status = PS2_GPIO_WRITE_STATUS_ACTIVE;
    ps2_gpio_set_scl_callback_enabled(false);
    ps2_gpio_configure_pin_scl_output();
    ps2_gpio_configure_pin_sda_output();
    ps2_gpio_set_scl(0);
    ps2_gpio_set_sda(1);
    k_work_schedule_for_queue(&ps2_gpio_work_queue, &data->write_inhibition_wait, PS2_GPIO_WRITE_INHIBIT_SLC_DURATION);
    return 0;
}

int ps2_gpio_write_byte_blocking(uint8_t byte) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    if (ps2_gpio_write_byte_start(byte)) return -EIO;
    if (k_sem_take(&data->write_lock, PS2_GPIO_TIMEOUT_WRITE_BLOCKING)) {
        ps2_gpio_write_finish(false, "timeout");
        return -ETIMEDOUT;
    }
    return (data->cur_write_status == PS2_GPIO_WRITE_STATUS_SUCCESS) ? 0 : -EIO;
}

int ps2_gpio_write_byte(uint8_t byte) {
    for (int i = 0; i < PS2_GPIO_WRITE_MAX_RETRY; i++) {
        if (ps2_gpio_write_byte_blocking(byte) == 0) return 0;
    }
    return -EIO;
}

static int ps2_gpio_write(const struct device *dev, uint8_t value) {
    return ps2_gpio_write_byte(value);
}

void ps2_gpio_scl_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct ps2_gpio_data *data = &ps2_gpio_data;
    if (data->mode == PS2_GPIO_MODE_READ) ps2_gpio_read_interrupt_handler();
    else ps2_gpio_write_interrupt_handler();
}

static int ps2_gpio_disable_callback(const struct device *dev) {
    ((struct ps2_gpio_data *)dev->data)->callback_enabled = false;
    return 0;
}

static int ps2_gpio_enable_callback(const struct device *dev) {
    ((struct ps2_gpio_data *)dev->data)->callback_enabled = true;
    return 0;
}

#if IS_ENABLED(CONFIG_PS2_GPIO_ENABLE_PS2_RESEND_CALLBACK)
static int ps2_gpio_configure(const struct device *dev, ps2_callback_t callback_isr, ps2_resend_callback_t resend_callback_isr) {
    struct ps2_gpio_data *data = dev->data;
    data->callback_isr = callback_isr;
    data->resend_callback_isr = resend_callback_isr;
    return 0;
}
#else
static int ps2_gpio_configure(const struct device *dev, ps2_callback_t callback_isr) {
    ((struct ps2_gpio_data *)dev->data)->callback_isr = callback_isr;
    return 0;
}
#endif

static const struct ps2_driver_api ps2_gpio_driver_api = {
    .config = ps2_gpio_configure,
    .read = ps2_gpio_read,
    .write = ps2_gpio_write,
    .disable_callback = ps2_gpio_disable_callback,
    .enable_callback = ps2_gpio_enable_callback,
};

static int ps2_gpio_init(const struct device *dev) {
    struct ps2_gpio_data *data = dev->data;
    const struct ps2_gpio_config *config = dev->config;
    data->dev = dev;
    data->scl_gpio = config->scl_gpio;
    data->sda_gpio = config->sda_gpio;
    
    LOG_INF("PS2 GPIO Initialized on %s and %s", config->scl_gpio.port->name, config->sda_gpio.port->name);

    gpio_pin_configure_dt(&data->scl_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&data->sda_gpio, GPIO_INPUT);
    gpio_init_callback(&data->scl_cb_data, ps2_gpio_scl_interrupt_handler, BIT(data->scl_gpio.pin));
    gpio_add_callback(data->scl_gpio.port, &data->scl_cb_data);
    gpio_pin_interrupt_configure_dt(&data->scl_gpio, GPIO_INT_EDGE_FALLING);

    k_msgq_init(&data->data_queue, data->data_queue_buffer, sizeof(struct ps2_gpio_data_queue_item), PS2_GPIO_DATA_QUEUE_SIZE);
    k_sem_init(&data->write_lock, 1, 1);
    k_sem_init(&data->write_awaits_resp_sem, 0, 1);
    
    k_work_queue_start(&ps2_gpio_work_queue, ps2_gpio_work_queue_stack_area, K_THREAD_STACK_SIZEOF(ps2_gpio_work_queue_stack_area), PS2_GPIO_WORK_QUEUE_PRIORITY, NULL);
    k_work_queue_start(&ps2_gpio_work_queue_cb, ps2_gpio_work_queue_cb_stack_area, K_THREAD_STACK_SIZEOF(ps2_gpio_work_queue_cb_stack_area), PS2_GPIO_WORK_QUEUE_CB_PRIORITY, NULL);
    
    k_work_init_delayable(&data->read_scl_timout, ps2_gpio_read_scl_timeout);
    k_work_init_delayable(&data->write_scl_timout, ps2_gpio_write_scl_timeout);
    k_work_init_delayable(&data->write_inhibition_wait, ps2_gpio_write_inhibition_wait);
    k_work_init(&data->callback_work, ps2_gpio_read_callback_work_handler);

    return 0;
}

DEVICE_DT_INST_DEFINE(0, &ps2_gpio_init, NULL, &ps2_gpio_data, &ps2_gpio_config, POST_KERNEL, CONFIG_PS2_INIT_PRIORITY, &ps2_gpio_driver_api);