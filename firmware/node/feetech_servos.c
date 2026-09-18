#include "feetech_servos.h"

#include "config.h"

#include <string.h>

#define FEETECH_MAX_BUSES 3

#ifndef FEETECH_MAX_SERVOS
#define FEETECH_MAX_SERVOS 8
#endif

#define FEETECH_CFG_SLOTS 8

#if (FEETECH_MAX_SERVOS < 1) || (FEETECH_MAX_SERVOS > FEETECH_CFG_SLOTS)
#error "FEETECH_MAX_SERVOS must be between 1 and 8"
#endif

#define FEETECH_BROADCAST_ID 0xFEU
#define FEETECH_INST_READ_DATA 0x02U
#define FEETECH_INST_SYNC_WRITE 0x83U
#define FEETECH_RAM_GOAL_POSITION_L 42U
#define FEETECH_GOAL_DATA_LEN 2U

#define FEETECH_TEMP_TO_KELVIN_OFFSET 273.15f
#define FEETECH_REG_PRESENT_POSITION 56U
#define FEETECH_REG_SPEED 58U
#define FEETECH_REG_LOAD 60U
#define FEETECH_REG_CURRENT 69U
#define FEETECH_REG_TEMPERATURE 63U
#define FEETECH_CURRENT_SCALE 0.001f
#define FEETECH_CURRENT_OFFSET 0.0f

struct feetech_servo_channel_t {
    uint8_t bus;
    uint8_t id;
    uint8_t index;
    uint16_t failsafe;
    uint16_t temperature_device_id;
    uint16_t last_pos;
    uint32_t timeout_count;
    uint32_t position_poll_count;
    bool enabled;
};

struct feetech_bus_t {
    UARTDriver *port;
    bool active;
};

struct feetech_servos_t {
    bool initialized;
    bool telemetry_enabled;
    bool current_temp_enabled;
    bool fast_speed_enabled;
    bool fast_load_enabled;
    uint32_t telemetry_delay_ms;
    uint32_t current_temp_divider;
    UARTConfig uart_cfg;
    mutex_t io_mutex;

    struct feetech_bus_t buses[FEETECH_MAX_BUSES + 1];
    struct feetech_servo_channel_t channels[FEETECH_MAX_SERVOS];
};

static struct feetech_servos_t feetech_servos = {0};
static bool feetech_telem_thread_started = false;

static const char *const feetech_busid_cfg_names[FEETECH_CFG_SLOTS] = {
    "FT1 bus+id", "FT2 bus+id", "FT3 bus+id", "FT4 bus+id", "FT5 bus+id", "FT6 bus+id", "FT7 bus+id", "FT8 bus+id"
};
static const char *const feetech_index_cfg_names[FEETECH_CFG_SLOTS] = {
    "FT1 index", "FT2 index", "FT3 index", "FT4 index", "FT5 index", "FT6 index", "FT7 index", "FT8 index"
};
static const char *const feetech_failsafe_cfg_names[FEETECH_CFG_SLOTS] = {
    "FT1 failsafe", "FT2 failsafe", "FT3 failsafe", "FT4 failsafe", "FT5 failsafe", "FT6 failsafe", "FT7 failsafe", "FT8 failsafe"
};

static inline uint16_t clamp_u16(uint32_t value, uint16_t max) {
    if (value > max) {
        return max;
    }
    return (uint16_t)value;
}

static int16_t feetech_decode_signed_15bit(uint16_t raw) {
    int16_t value = (int16_t)(raw & 0x7FFFU);
    if ((raw & 0x8000U) != 0U) {
        value = (int16_t)(-value);
    }
    return value;
}

static float feetech_decode_signed_load(uint16_t raw) {
    int16_t magnitude = (int16_t)(raw & 0x03FFU);
    if ((raw & 0x0400U) != 0U) {
        magnitude = (int16_t)(-magnitude);
    }
    return (float)magnitude / 1000.0f;
}

static uint16_t raw_to_sts_position(int16_t raw_cmd) {
    int32_t shifted = (int32_t)raw_cmd + 8192;

    if (shifted < 0) {
        shifted = 0;
    } else if (shifted > 16383) {
        shifted = 16383;
    }

    return (uint16_t)((shifted * 4095) / 16383);
}

static uint8_t feetech_checksum(const uint8_t *frame, uint8_t last_idx) {
    uint16_t sum = 0;

    for (uint8_t i = 2; i <= last_idx; i++) {
        sum += frame[i];
    }

    return (uint8_t)(~sum);
}

static UARTDriver *feetech_port_from_idx(uint8_t bus_idx) {
    if (bus_idx == 1U) {
        return &UARTD1;
    }
    if (bus_idx == 2U) {
        return &UARTD2;
    }
    if (bus_idx == 3U) {
        return &UARTD3;
    }
    return NULL;
}

static void feetech_configure_port_lines(uint8_t bus_idx) {
    if (bus_idx == 1U) {
        palSetLineMode(SERIAL1_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    } else if (bus_idx == 2U) {
        palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    } else if (bus_idx == 3U) {
        palSetLineMode(SERIAL3_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    }
}

static uint8_t feetech_active_channel_count(void) {
    uint8_t active = 0;

    for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
        if (feetech_servos.channels[i].enabled) {
            active++;
        }
    }

    return active;
}

static bool feetech_load_channel_from_config(uint8_t cfg_idx, struct feetech_servo_channel_t *out) {
    uint16_t bus_id;
    uint8_t bus;
    uint8_t id;
    uint8_t index;

    if (out == NULL || cfg_idx >= FEETECH_CFG_SLOTS) {
        return false;
    }

    bus_id = clamp_u16((uint32_t)config_get_by_name((char *)feetech_busid_cfg_names[cfg_idx], 0)->val.i, 3253);
    bus = (uint8_t)(bus_id / 1000U);
    id = (uint8_t)(bus_id % 1000U);
    index = (uint8_t)config_get_by_name((char *)feetech_index_cfg_names[cfg_idx], 0)->val.i;

    if (bus < 1U || bus > FEETECH_MAX_BUSES || id == 0U || id >= FEETECH_BROADCAST_ID || index == 0xFFU) {
        return false;
    }

    out->id = id;
    out->index = index;
    out->bus = bus;
    out->timeout_count = 0;
    out->position_poll_count = 0;
    out->last_pos = 0xFFFFU;
    out->failsafe = clamp_u16((uint32_t)config_get_by_name((char *)feetech_failsafe_cfg_names[cfg_idx], 0)->val.i, 4095);
    out->temperature_device_id = (uint16_t)(1000U + out->id);

    out->enabled = true;
    return true;
}

static void feetech_send_targets_bus(uint8_t bus_idx, const uint16_t positions[FEETECH_MAX_SERVOS], bool force_send) {
    struct feetech_bus_t *bus = &feetech_servos.buses[bus_idx];
    if (!feetech_servos.initialized || !bus->active || bus->port == NULL) {
        return;
    }

    uint8_t active = 0;
    for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
        if (feetech_servos.channels[i].enabled && feetech_servos.channels[i].bus == bus_idx) {
            active++;
        }
    }

    if (active == 0) {
        return;
    }

    bool changed = force_send;
    if (!changed) {
        for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
            if (!feetech_servos.channels[i].enabled || feetech_servos.channels[i].bus != bus_idx) {
                continue;
            }

            if (feetech_servos.channels[i].last_pos != positions[i]) {
                changed = true;
                break;
            }
        }
    }

    if (!changed) {
        return;
    }

    uint8_t frame[96];
    uint8_t idx = 0;
    frame[idx++] = 0xFF;
    frame[idx++] = 0xFF;
    frame[idx++] = FEETECH_BROADCAST_ID;

    const uint8_t params_len = 2U + (uint8_t)(active * (1U + FEETECH_GOAL_DATA_LEN));
    frame[idx++] = (uint8_t)(params_len + 2U);
    frame[idx++] = FEETECH_INST_SYNC_WRITE;
    frame[idx++] = FEETECH_RAM_GOAL_POSITION_L;
    frame[idx++] = FEETECH_GOAL_DATA_LEN;

    for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
        if (!feetech_servos.channels[i].enabled || feetech_servos.channels[i].bus != bus_idx) {
            continue;
        }

        const uint16_t pos = clamp_u16(positions[i], 4095);

        frame[idx++] = feetech_servos.channels[i].id;
        frame[idx++] = (uint8_t)(pos & 0xFF);
        frame[idx++] = (uint8_t)((pos >> 8) & 0xFF);

        feetech_servos.channels[i].last_pos = pos;
    }

    frame[idx] = feetech_checksum(frame, (uint8_t)(idx - 1));
    idx++;

    size_t tx_size = idx;
    chMtxLock(&feetech_servos.io_mutex);
    uartSendFullTimeout(bus->port, &tx_size, frame, TIME_MS2I(20));
    chMtxUnlock(&feetech_servos.io_mutex);
}

static bool feetech_read_data(struct feetech_bus_t *bus, uint8_t servo_id, uint8_t reg_addr, uint8_t data_len, uint8_t *out_data) {
    if (bus == NULL || bus->port == NULL || !bus->active || out_data == NULL || data_len == 0U) {
        return false;
    }

    uint8_t req[8];
    req[0] = 0xFF;
    req[1] = 0xFF;
    req[2] = servo_id;
    req[3] = 4;
    req[4] = FEETECH_INST_READ_DATA;
    req[5] = reg_addr;
    req[6] = data_len;
    req[7] = feetech_checksum(req, 6);

    uint8_t rx[32];
    size_t dummy = sizeof(rx);
    size_t tx_size = sizeof(req);
    size_t rx_size = sizeof(rx);

    chMtxLock(&feetech_servos.io_mutex);
    uartReceiveTimeout(bus->port, &dummy, rx, TIME_IMMEDIATE);
    uartSendFullTimeout(bus->port, &tx_size, req, TIME_MS2I(10));
    msg_t status = uartReceiveTimeout(bus->port, &rx_size, rx, TIME_MS2I(15));
    chMtxUnlock(&feetech_servos.io_mutex);

    if (status != MSG_OK || rx_size < 6U) {
        return false;
    }

    for (size_t i = 0; i + 5U < rx_size; i++) {
        if (rx[i] != 0xFF || rx[i + 1U] != 0xFF || rx[i + 2U] != servo_id) {
            continue;
        }

        const uint8_t frame_len = rx[i + 3U];
        const size_t total_len = (size_t)frame_len + 4U;
        if (i + total_len > rx_size || frame_len < 2U) {
            continue;
        }

        const uint8_t checksum = feetech_checksum(&rx[i], (uint8_t)(total_len - 2U));
        if (checksum != rx[i + total_len - 1U]) {
            continue;
        }

        const uint8_t err = rx[i + 4U];
        const uint8_t payload_len = (uint8_t)(frame_len - 2U);
        if (err != 0U || payload_len < data_len) {
            return false;
        }

        memcpy(out_data, &rx[i + 5U], data_len);
        return true;
    }

    return false;
}

static void feetech_publish_status(const struct feetech_servo_channel_t *ch, bool has_position, uint16_t position_raw,
                                   bool has_speed, float speed, int16_t speed_raw,
                                   bool has_load, float load,
                                   bool has_current, float current,
                                   bool has_temperature, float temperature_c) {
    static uint8_t actuator_transfer_id;
    static uint8_t esc_transfer_id;
    static uint8_t temp_transfer_id;

    if (ch == NULL) {
        return;
    }

    if (has_position) {
        struct uavcan_equipment_actuator_Status status;
        status.actuator_id = ch->id;
        status.position = ((float)position_raw / 4095.0f) * 2.0f - 1.0f;
        status.force = has_load ? load : (has_current ? current : 0.0f);
        status.speed = has_speed ? speed : 0.0f;
        status.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;

        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_actuator_Status_encode(&status, buffer);
        uavcanBroadcastAll(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                           UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                           &actuator_transfer_id,
                           CANARD_TRANSFER_PRIORITY_LOW,
                           buffer,
                           total_size);
    }

    if (has_current || has_temperature || has_speed) {
        struct uavcan_equipment_esc_Status esc_status;
        esc_status.error_count = ch->timeout_count;
        esc_status.voltage = 0.0f;
        esc_status.current = has_current ? current : 0.0f;
        esc_status.temperature = has_temperature ? (temperature_c + FEETECH_TEMP_TO_KELVIN_OFFSET) : 0.0f;
        esc_status.rpm = has_speed ? speed_raw : 0;
        esc_status.power_rating_pct = 0;
        esc_status.esc_index = ch->id;

        uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_esc_Status_encode(&esc_status, buffer);
        uavcanBroadcastAll(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                           UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                           &esc_transfer_id,
                           CANARD_TRANSFER_PRIORITY_LOW,
                           buffer,
                           total_size);
    }

    if (has_temperature) {
        struct uavcan_equipment_device_Temperature temperature;
        temperature.device_id = (ch->temperature_device_id != 0U) ? ch->temperature_device_id : (uint16_t)(1000U + ch->id);
        temperature.temperature = temperature_c + FEETECH_TEMP_TO_KELVIN_OFFSET;
        temperature.error_flags = 0;

        uint8_t buffer[UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_device_Temperature_encode(&temperature, buffer);
        uavcanBroadcastAll(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE,
                           UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID,
                           &temp_transfer_id,
                           CANARD_TRANSFER_PRIORITY_LOW,
                           buffer,
                           total_size);
    }
}

static void feetech_poll_channel(struct feetech_servo_channel_t *ch) {
    struct feetech_bus_t *bus;
    uint8_t data[2];
    bool poll_current_temp;
    bool has_position = false;
    bool has_speed = false;
    bool has_load = false;
    bool has_current = false;
    bool has_temperature = false;
    uint16_t position_raw = 0;
    int16_t speed_raw = 0;
    float speed = 0.0f;
    float load = 0.0f;
    float current = 0.0f;
    float temperature_c = 0.0f;

    if (ch == NULL || !ch->enabled || ch->bus == 0U || ch->bus > FEETECH_MAX_BUSES) {
        return;
    }

    bus = &feetech_servos.buses[ch->bus];
    if (!bus->active || bus->port == NULL) {
        return;
    }

    poll_current_temp = feetech_servos.current_temp_enabled;
    if (poll_current_temp && feetech_servos.current_temp_divider > 1U) {
        poll_current_temp = ((ch->position_poll_count % feetech_servos.current_temp_divider) == 0U);
    }
    ch->position_poll_count++;

    if (feetech_read_data(bus, ch->id, FEETECH_REG_PRESENT_POSITION, 2, data)) {
        position_raw = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
        has_position = true;
    }

    if (feetech_servos.fast_speed_enabled && feetech_read_data(bus, ch->id, FEETECH_REG_SPEED, 2, data)) {
        speed_raw = feetech_decode_signed_15bit((uint16_t)(data[0] | ((uint16_t)data[1] << 8)));
        speed = (float)speed_raw;
        has_speed = true;
    }

    if (feetech_servos.fast_load_enabled && feetech_read_data(bus, ch->id, FEETECH_REG_LOAD, 2, data)) {
        load = feetech_decode_signed_load((uint16_t)(data[0] | ((uint16_t)data[1] << 8)));
        has_load = true;
    }

    if (poll_current_temp && feetech_read_data(bus, ch->id, FEETECH_REG_CURRENT, 2, data)) {
        int16_t raw_current = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
        current = (raw_current * FEETECH_CURRENT_SCALE) + FEETECH_CURRENT_OFFSET;
        has_current = true;
    }

    if (poll_current_temp && feetech_read_data(bus, ch->id, FEETECH_REG_TEMPERATURE, 1, data)) {
        temperature_c = (float)((int8_t)data[0]);
        has_temperature = true;
    }

    if (!has_position && !has_speed && !has_load && !has_current && !has_temperature) {
        ch->timeout_count++;
        return;
    }

    feetech_publish_status(ch, has_position, position_raw,
                           has_speed, speed, speed_raw,
                           has_load, load,
                           has_current, current,
                           has_temperature, temperature_c);
}

static THD_FUNCTION(feetech_telem_thd, arg) {
    (void)arg;
    chRegSetThreadName("ft_telem");

    while (true) {
        if (!feetech_servos.initialized || !feetech_servos.telemetry_enabled) {
            chThdSleepMilliseconds(200);
            continue;
        }

        uint8_t active_channels = feetech_active_channel_count();
        if (active_channels == 0U) {
            chThdSleepMilliseconds(200);
            continue;
        }

        uint32_t delay_ms = feetech_servos.telemetry_delay_ms / active_channels;
        if (delay_ms < 5U) {
            delay_ms = 5U;
        }

        for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
            if (feetech_servos.channels[i].enabled) {
                feetech_poll_channel(&feetech_servos.channels[i]);
                chThdSleepMilliseconds(delay_ms);
            }
        }
    }
}

void feetech_servos_apply_rawcommand(const struct uavcan_equipment_esc_RawCommand *msg) {
    uint16_t targets[FEETECH_MAX_SERVOS] = {0};

    if (!feetech_servos.initialized || msg == NULL) {
        return;
    }

    for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
        if (!feetech_servos.channels[i].enabled) {
            continue;
        }

        if (feetech_servos.channels[i].index < msg->cmd.len) {
            targets[i] = raw_to_sts_position(msg->cmd.data[feetech_servos.channels[i].index]);
        } else {
            targets[i] = feetech_servos.channels[i].failsafe;
        }
    }

    for (uint8_t bus_idx = 1; bus_idx <= FEETECH_MAX_BUSES; bus_idx++) {
        feetech_send_targets_bus(bus_idx, targets, false);
    }
}

void feetech_servos_set_failsafe(void) {
    uint16_t targets[FEETECH_MAX_SERVOS] = {0};

    for (uint8_t i = 0; i < FEETECH_MAX_SERVOS; i++) {
        if (feetech_servos.channels[i].enabled) {
            targets[i] = feetech_servos.channels[i].failsafe;
        }
    }

    for (uint8_t bus_idx = 1; bus_idx <= FEETECH_MAX_BUSES; bus_idx++) {
        feetech_send_targets_bus(bus_idx, targets, true);
    }
}

void feetech_servos_disable(void) {
    feetech_servos.initialized = false;
}

void feetech_servos_init(void) {
    memset(&feetech_servos, 0, sizeof(feetech_servos));

    chMtxObjectInit(&feetech_servos.io_mutex);

    float telem_freq = config_get_by_name("FT telem position frequency", 0)->val.f;
    float current_temp_freq = config_get_by_name("FT telem current/temp frequency", 0)->val.f;
    feetech_servos.fast_speed_enabled = (config_get_by_name("FT telem speed enable", 0)->val.i != 0);
    feetech_servos.fast_load_enabled = (config_get_by_name("FT telem load enable", 0)->val.i != 0);
    if (telem_freq > 0.0f) {
        feetech_servos.telemetry_enabled = true;
        feetech_servos.telemetry_delay_ms = (uint32_t)(1000.0f / telem_freq);
        if (feetech_servos.telemetry_delay_ms == 0U) {
            feetech_servos.telemetry_delay_ms = 1U;
        }

        if (current_temp_freq > 0.0f) {
            feetech_servos.current_temp_enabled = true;
            if (current_temp_freq >= telem_freq) {
                feetech_servos.current_temp_divider = 1U;
            } else {
                feetech_servos.current_temp_divider = (uint32_t)(telem_freq / current_temp_freq);
                if (feetech_servos.current_temp_divider == 0U) {
                    feetech_servos.current_temp_divider = 1U;
                } else if (((float)feetech_servos.current_temp_divider * current_temp_freq) < telem_freq) {
                    feetech_servos.current_temp_divider++;
                }
            }
        }
    }

    for (uint8_t i = 1; i <= FEETECH_MAX_BUSES; i++) {
        feetech_servos.buses[i].active = false;
    }

    feetech_servos.uart_cfg.speed = config_get_by_name("FT baud", 0)->val.i;
    feetech_servos.uart_cfg.cr1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
    feetech_servos.uart_cfg.cr2 = 0;
    feetech_servos.uart_cfg.cr3 = USART_CR3_HDSEL;

    uint8_t channel_idx = 0;
    for (uint8_t cfg_idx = 0; cfg_idx < FEETECH_CFG_SLOTS && channel_idx < FEETECH_MAX_SERVOS; cfg_idx++) {
        struct feetech_servo_channel_t cfg = {0};
        if (feetech_load_channel_from_config(cfg_idx, &cfg)) {
            feetech_servos.channels[channel_idx++] = cfg;
            feetech_servos.buses[cfg.bus].active = true;
        }
    }

    for (uint8_t i = 1; i <= FEETECH_MAX_BUSES; i++) {
        struct feetech_bus_t *bus = &feetech_servos.buses[i];

        if (!bus->active) {
            continue;
        }

        feetech_configure_port_lines(i);
        bus->port = feetech_port_from_idx(i);
        if (bus->port != NULL) {
            uartStart(bus->port, &feetech_servos.uart_cfg);
        } else {
            bus->active = false;
        }
    }

    feetech_servos.initialized = (feetech_active_channel_count() > 0U);
    if (feetech_servos.initialized) {
        feetech_servos_set_failsafe();
    }

    if (!feetech_telem_thread_started) {
        chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "ft_telem", NORMALPRIO - 7, feetech_telem_thd, NULL);
        feetech_telem_thread_started = true;
    }
}
