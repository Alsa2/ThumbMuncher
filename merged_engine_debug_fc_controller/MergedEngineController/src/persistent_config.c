#include "persistent_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "board_config.h"
#include "engine_control.h"
#include "sensors.h"
#include "custom_can_node.h"
#include "custom_can_protocol.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#ifndef FRAM_I2C_ADDR
#define FRAM_I2C_ADDR 0x50u
#endif

#ifndef FRAM_CONFIG_ADDR
#define FRAM_CONFIG_ADDR 0x0000u
#endif

#define FRAM_CONFIG_MAGIC   0x454D4346u  // 'EMCF'
#define FRAM_CONFIG_VERSION 5u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc32;
    uint32_t save_counter;

    float idle_rpm;
    float max_rpm;
    uint16_t idle_us;
    uint16_t max_us;

    float kp_us_per_rpm;
    float ki_us_per_rpm_s;
    float kd_us_per_rpm_per_s;
    float correction_limit_us;

    uint16_t hall_threshold_high_raw;
    uint16_t hall_threshold_low_raw;

    uint32_t board_id;
    uint16_t hall_cal_min_raw;
    uint16_t hall_cal_max_raw;
    uint8_t reserved[16];
} PersistentConfigRecordV3;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc32;
    uint32_t save_counter;

    float idle_rpm;
    float max_rpm;
    uint16_t idle_us;
    uint16_t max_us;

    float kp_us_per_rpm;
    float ki_us_per_rpm_s;
    float kd_us_per_rpm_per_s;
    float correction_limit_us;

    uint16_t hall_threshold_high_raw;
    uint16_t hall_threshold_low_raw;

    uint32_t board_id;
    uint16_t hall_cal_min_raw;
    uint16_t hall_cal_max_raw;

    // Runtime-configurable startup priming values. start_us is commanded exactly
    // while waiting for first RPM; start_hold_ms is the additional hold time
    // after RPM is detected. Both are saved with the PID/feedforward record.
    uint16_t start_us;
    uint16_t start_hold_ms;
    uint8_t reserved[12];
} PersistentConfigRecordV4;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc32;
    uint32_t save_counter;

    float idle_rpm;
    float max_rpm;
    uint16_t idle_us;
    uint16_t max_us;

    float kp_us_per_rpm;
    float ki_us_per_rpm_s;
    float kd_us_per_rpm_per_s;
    float correction_limit_us;

    uint16_t hall_threshold_high_raw;
    uint16_t hall_threshold_low_raw;

    uint32_t board_id;
    uint16_t hall_cal_min_raw;
    uint16_t hall_cal_max_raw;
    uint16_t start_us;
    uint16_t start_hold_ms;

    // Version 5 adds the atomically committed feedforward fit model.
    EngineFeedforwardModelConfig feedforward_model;
    uint8_t reserved[8];
} PersistentConfigRecord;

#ifndef FRAM_BOARD_ID_ADDR
#define FRAM_BOARD_ID_ADDR 0x0100u
#endif

_Static_assert(sizeof(PersistentConfigRecord) <= FRAM_BOARD_ID_ADDR,
               "Persistent config record overlaps the dedicated FRAM board-ID record");

#define FRAM_BOARD_ID_MAGIC   0x454D4944u  // 'EMID'
#define FRAM_BOARD_ID_VERSION 1u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc32;
    uint32_t save_counter;
    uint32_t board_id;
    uint8_t reserved[8];
} PersistentBoardIdRecord;

static bool g_fram_present = false;
static bool g_last_load_valid = false;
static uint32_t g_save_counter = 0u;
static uint32_t g_loaded_board_id = 0u;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            const uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t record_crc(const PersistentConfigRecord *rec)
{
    PersistentConfigRecord tmp = *rec;
    tmp.crc32 = 0u;
    return crc32_update(0u, (const uint8_t *)&tmp, sizeof(tmp));
}

static uint32_t record_v4_crc(const PersistentConfigRecordV4 *rec)
{
    PersistentConfigRecordV4 tmp = *rec;
    tmp.crc32 = 0u;
    return crc32_update(0u, (const uint8_t *)&tmp, sizeof(tmp));
}

static uint32_t record_v3_crc(const PersistentConfigRecordV3 *rec)
{
    PersistentConfigRecordV3 tmp = *rec;
    tmp.crc32 = 0u;
    return crc32_update(0u, (const uint8_t *)&tmp, sizeof(tmp));
}

static uint32_t board_id_record_crc(const PersistentBoardIdRecord *rec)
{
    PersistentBoardIdRecord tmp = *rec;
    tmp.crc32 = 0u;
    return crc32_update(0u, (const uint8_t *)&tmp, sizeof(tmp));
}

static bool board_id_is_valid(uint32_t board_id)
{
    return board_id != 0u && board_id != CUSTOM_CAN_BROADCAST_BOARD_ID;
}

static bool fram_read(uint16_t addr, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0u) {
        return true;
    }
    uint8_t a[2] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFFu)};
    int wr = i2c_write_timeout_us(FRAM_I2C_PORT, FRAM_I2C_ADDR, a, sizeof(a), true, 5000u);
    if (wr != (int)sizeof(a)) {
        return false;
    }
    int rd = i2c_read_timeout_us(FRAM_I2C_PORT, FRAM_I2C_ADDR, data, len, false, 5000u);
    return rd == (int)len;
}

static bool fram_write(uint16_t addr, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0u) {
        return true;
    }

    // FM24CL64B is FRAM, not EEPROM, so there is no erase delay. Keep chunks
    // small so the I2C transaction comfortably fits the RP2040 SDK buffers.
    while (len > 0u) {
        const size_t chunk = (len > 16u) ? 16u : len;
        uint8_t buf[2 + 16];
        buf[0] = (uint8_t)(addr >> 8);
        buf[1] = (uint8_t)(addr & 0xFFu);
        memcpy(&buf[2], data, chunk);
        int wr = i2c_write_timeout_us(FRAM_I2C_PORT, FRAM_I2C_ADDR, buf, 2u + chunk, false, 5000u);
        if (wr != (int)(2u + chunk)) {
            return false;
        }
        addr = (uint16_t)(addr + chunk);
        data += chunk;
        len -= chunk;
    }
    return true;
}

static bool validate_board_id_record(const PersistentBoardIdRecord *rec)
{
    if (rec == NULL) return false;
    if (rec->magic != FRAM_BOARD_ID_MAGIC) return false;
    if (rec->version != FRAM_BOARD_ID_VERSION) return false;
    if (rec->size != sizeof(PersistentBoardIdRecord)) return false;
    if (board_id_record_crc(rec) != rec->crc32) return false;
    return board_id_is_valid(rec->board_id);
}

static bool load_board_id_record(uint32_t *out_board_id)
{
    if (out_board_id == NULL || !g_fram_present) {
        return false;
    }
    PersistentBoardIdRecord rec;
    memset(&rec, 0, sizeof(rec));
    if (!fram_read(FRAM_BOARD_ID_ADDR, (uint8_t *)&rec, sizeof(rec))) {
        g_fram_present = false;
        return false;
    }
    if (!validate_board_id_record(&rec)) {
        return false;
    }
    *out_board_id = rec.board_id;
    return true;
}

static bool save_board_id_record(uint32_t board_id)
{
    if (!g_fram_present || !board_id_is_valid(board_id)) {
        return false;
    }

    PersistentBoardIdRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = FRAM_BOARD_ID_MAGIC;
    rec.version = FRAM_BOARD_ID_VERSION;
    rec.size = sizeof(rec);
    rec.save_counter = g_save_counter + 1u;
    rec.board_id = board_id;
    rec.crc32 = board_id_record_crc(&rec);

    if (!fram_write(FRAM_BOARD_ID_ADDR, (const uint8_t *)&rec, sizeof(rec))) {
        g_fram_present = false;
        return false;
    }

    PersistentBoardIdRecord verify;
    memset(&verify, 0, sizeof(verify));
    if (!fram_read(FRAM_BOARD_ID_ADDR, (uint8_t *)&verify, sizeof(verify))) {
        g_fram_present = false;
        return false;
    }
    if (!validate_board_id_record(&verify) || verify.board_id != board_id || verify.crc32 != rec.crc32) {
        return false;
    }

    g_loaded_board_id = board_id;
    return true;
}


static bool validate_record_v3(const PersistentConfigRecordV3 *rec)
{
    if (rec == NULL) return false;
    if (rec->magic != FRAM_CONFIG_MAGIC) return false;
    if (rec->version != 3u) return false;
    if (rec->size != sizeof(PersistentConfigRecordV3)) return false;
    if (record_v3_crc(rec) != rec->crc32) return false;

    EngineControlRuntimeConfig cfg = {
        .idle_rpm = rec->idle_rpm,
        .max_rpm = rec->max_rpm,
        .idle_us = rec->idle_us,
        .max_us = rec->max_us,
        .kp_us_per_rpm = rec->kp_us_per_rpm,
        .ki_us_per_rpm_s = rec->ki_us_per_rpm_s,
        .kd_us_per_rpm_per_s = rec->kd_us_per_rpm_per_s,
        .correction_limit_us = rec->correction_limit_us,
        .start_us = THROTTLE_START_US,
        .start_hold_ms = START_HOLD_AFTER_RPM_MS,
    };
    engine_control_make_default_feedforward_model(&cfg.feedforward_model, cfg.idle_us, cfg.max_us);
    if (!engine_control_runtime_config_is_valid(&cfg)) return false;
    if (!sensors_hall_thresholds_are_valid(rec->hall_threshold_high_raw, rec->hall_threshold_low_raw)) return false;
    if (rec->board_id != 0u && !board_id_is_valid(rec->board_id)) return false;
    if ((rec->hall_cal_min_raw != 0u || rec->hall_cal_max_raw != 0u) &&
        !sensors_hall_calibration_is_valid(rec->hall_cal_min_raw, rec->hall_cal_max_raw)) {
        return false;
    }
    return true;
}

static bool validate_record_v4(const PersistentConfigRecordV4 *rec)
{
    if (rec == NULL) return false;
    if (rec->magic != FRAM_CONFIG_MAGIC) return false;
    if (rec->version != 4u) return false;
    if (rec->size != sizeof(PersistentConfigRecordV4)) return false;
    if (record_v4_crc(rec) != rec->crc32) return false;

    EngineControlRuntimeConfig cfg = {
        .idle_rpm = rec->idle_rpm,
        .max_rpm = rec->max_rpm,
        .idle_us = rec->idle_us,
        .max_us = rec->max_us,
        .kp_us_per_rpm = rec->kp_us_per_rpm,
        .ki_us_per_rpm_s = rec->ki_us_per_rpm_s,
        .kd_us_per_rpm_per_s = rec->kd_us_per_rpm_per_s,
        .correction_limit_us = rec->correction_limit_us,
        .start_us = rec->start_us,
        .start_hold_ms = rec->start_hold_ms,
    };
    engine_control_make_default_feedforward_model(&cfg.feedforward_model, cfg.idle_us, cfg.max_us);
    if (!engine_control_runtime_config_is_valid(&cfg)) {
        return false;
    }
    if (!sensors_hall_thresholds_are_valid(rec->hall_threshold_high_raw, rec->hall_threshold_low_raw)) {
        return false;
    }
    if (rec->board_id != 0u && !board_id_is_valid(rec->board_id)) {
        return false;
    }
    if ((rec->hall_cal_min_raw != 0u || rec->hall_cal_max_raw != 0u) &&
        !sensors_hall_calibration_is_valid(rec->hall_cal_min_raw, rec->hall_cal_max_raw)) {
        return false;
    }
    return true;
}

static bool validate_record(const PersistentConfigRecord *rec)
{
    if (rec == NULL) return false;
    if (rec->magic != FRAM_CONFIG_MAGIC) return false;
    if (rec->version != FRAM_CONFIG_VERSION) return false;
    if (rec->size != sizeof(PersistentConfigRecord)) return false;
    if (record_crc(rec) != rec->crc32) return false;

    EngineControlRuntimeConfig cfg = {
        .idle_rpm = rec->idle_rpm,
        .max_rpm = rec->max_rpm,
        .idle_us = rec->idle_us,
        .max_us = rec->max_us,
        .kp_us_per_rpm = rec->kp_us_per_rpm,
        .ki_us_per_rpm_s = rec->ki_us_per_rpm_s,
        .kd_us_per_rpm_per_s = rec->kd_us_per_rpm_per_s,
        .correction_limit_us = rec->correction_limit_us,
        .start_us = rec->start_us,
        .start_hold_ms = rec->start_hold_ms,
        .feedforward_model = rec->feedforward_model,
    };
    if (!engine_control_runtime_config_is_valid(&cfg)) return false;
    if (!sensors_hall_thresholds_are_valid(rec->hall_threshold_high_raw, rec->hall_threshold_low_raw)) return false;
    if (rec->board_id != 0u && !board_id_is_valid(rec->board_id)) return false;
    if ((rec->hall_cal_min_raw != 0u || rec->hall_cal_max_raw != 0u) &&
        !sensors_hall_calibration_is_valid(rec->hall_cal_min_raw, rec->hall_cal_max_raw)) return false;
    return true;
}

bool persistent_config_init(void)
{
    i2c_init(FRAM_I2C_PORT, FRAM_I2C_BAUD_HZ);
    gpio_set_function(PIN_FRAM_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_FRAM_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_FRAM_I2C_SDA);
    gpio_pull_up(PIN_FRAM_I2C_SCL);

    uint8_t probe = 0u;
    g_fram_present = fram_read(0x0000u, &probe, 1u);
    return g_fram_present;
}

bool persistent_config_load_into_runtime(void)
{
    g_last_load_valid = false;
    g_loaded_board_id = 0u;
    if (!g_fram_present) {
        return false;
    }

    PersistentConfigRecord rec;
    memset(&rec, 0, sizeof(rec));
    if (!fram_read(FRAM_CONFIG_ADDR, (uint8_t *)&rec, sizeof(rec))) {
        g_fram_present = false;
        return false;
    }
    if (!validate_record(&rec)) {
        PersistentConfigRecordV4 rec4;
        memset(&rec4, 0, sizeof(rec4));
        if (fram_read(FRAM_CONFIG_ADDR, (uint8_t *)&rec4, sizeof(rec4)) && validate_record_v4(&rec4)) {
            EngineControlRuntimeConfig cfg4 = {
                .idle_rpm = rec4.idle_rpm, .max_rpm = rec4.max_rpm,
                .idle_us = rec4.idle_us, .max_us = rec4.max_us,
                .kp_us_per_rpm = rec4.kp_us_per_rpm, .ki_us_per_rpm_s = rec4.ki_us_per_rpm_s,
                .kd_us_per_rpm_per_s = rec4.kd_us_per_rpm_per_s, .correction_limit_us = rec4.correction_limit_us,
                .start_us = rec4.start_us, .start_hold_ms = rec4.start_hold_ms,
            };
            engine_control_make_default_feedforward_model(&cfg4.feedforward_model, cfg4.idle_us, cfg4.max_us);
            if (!engine_control_apply_runtime_config(&cfg4)) return false;
            if (!sensors_apply_hall_thresholds_raw(rec4.hall_threshold_high_raw, rec4.hall_threshold_low_raw)) return false;
            if (sensors_hall_calibration_is_valid(rec4.hall_cal_min_raw, rec4.hall_cal_max_raw))
                (void)sensors_apply_hall_calibration_raw(rec4.hall_cal_min_raw, rec4.hall_cal_max_raw);
            g_save_counter = rec4.save_counter;
            uint32_t fallback_board_id_v4 = 0u;
            g_loaded_board_id = load_board_id_record(&fallback_board_id_v4) ? fallback_board_id_v4 : rec4.board_id;
            g_last_load_valid = true;
            return true;
        }

        PersistentConfigRecordV3 rec3;
        memset(&rec3, 0, sizeof(rec3));
        if (fram_read(FRAM_CONFIG_ADDR, (uint8_t *)&rec3, sizeof(rec3)) &&
            validate_record_v3(&rec3)) {
            EngineControlRuntimeConfig cfg3 = {
                .idle_rpm = rec3.idle_rpm,
                .max_rpm = rec3.max_rpm,
                .idle_us = rec3.idle_us,
                .max_us = rec3.max_us,
                .kp_us_per_rpm = rec3.kp_us_per_rpm,
                .ki_us_per_rpm_s = rec3.ki_us_per_rpm_s,
                .kd_us_per_rpm_per_s = rec3.kd_us_per_rpm_per_s,
                .correction_limit_us = rec3.correction_limit_us,
                .start_us = THROTTLE_START_US,
                .start_hold_ms = START_HOLD_AFTER_RPM_MS,
            };
            engine_control_make_default_feedforward_model(&cfg3.feedforward_model, cfg3.idle_us, cfg3.max_us);
            if (!engine_control_apply_runtime_config(&cfg3)) return false;
            if (!sensors_apply_hall_thresholds_raw(rec3.hall_threshold_high_raw, rec3.hall_threshold_low_raw)) return false;
            if (sensors_hall_calibration_is_valid(rec3.hall_cal_min_raw, rec3.hall_cal_max_raw)) {
                (void)sensors_apply_hall_calibration_raw(rec3.hall_cal_min_raw, rec3.hall_cal_max_raw);
            }
            g_save_counter = rec3.save_counter;
            uint32_t fallback_board_id_v3 = 0u;
            if (load_board_id_record(&fallback_board_id_v3)) {
                g_loaded_board_id = fallback_board_id_v3;
            } else {
                g_loaded_board_id = rec3.board_id;
            }
            g_last_load_valid = true;
            return true;
        }

        uint32_t fallback_board_id = 0u;
        if (load_board_id_record(&fallback_board_id)) {
            g_loaded_board_id = fallback_board_id;
        }
        return false;
    }

    EngineControlRuntimeConfig cfg = {
        .idle_rpm = rec.idle_rpm,
        .max_rpm = rec.max_rpm,
        .idle_us = rec.idle_us,
        .max_us = rec.max_us,
        .kp_us_per_rpm = rec.kp_us_per_rpm,
        .ki_us_per_rpm_s = rec.ki_us_per_rpm_s,
        .kd_us_per_rpm_per_s = rec.kd_us_per_rpm_per_s,
        .correction_limit_us = rec.correction_limit_us,
        .start_us = rec.start_us,
        .start_hold_ms = rec.start_hold_ms,
        .feedforward_model = rec.feedforward_model,
    };

    if (!engine_control_apply_runtime_config(&cfg)) {
        return false;
    }
    if (!sensors_apply_hall_thresholds_raw(rec.hall_threshold_high_raw, rec.hall_threshold_low_raw)) {
        return false;
    }
    if (sensors_hall_calibration_is_valid(rec.hall_cal_min_raw, rec.hall_cal_max_raw)) {
        (void)sensors_apply_hall_calibration_raw(rec.hall_cal_min_raw, rec.hall_cal_max_raw);
    }

    g_save_counter = rec.save_counter;
    uint32_t fallback_board_id = 0u;
    if (load_board_id_record(&fallback_board_id)) {
        g_loaded_board_id = fallback_board_id;
    } else {
        g_loaded_board_id = rec.board_id;
    }
    g_last_load_valid = true;
    return true;
}

bool persistent_config_save_from_runtime(void)
{
    if (!g_fram_present) {
        return false;
    }

    EngineControlRuntimeConfig cfg;
    engine_control_get_runtime_config(&cfg);

    uint16_t hall_high = 0u;
    uint16_t hall_low = 0u;
    uint16_t hall_cal_min = 0u;
    uint16_t hall_cal_max = 0u;
    sensors_get_hall_thresholds_raw(&hall_high, &hall_low);
    sensors_get_hall_calibration_raw(&hall_cal_min, &hall_cal_max);

    PersistentConfigRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = FRAM_CONFIG_MAGIC;
    rec.version = FRAM_CONFIG_VERSION;
    rec.size = sizeof(rec);
    rec.save_counter = g_save_counter + 1u;
    rec.idle_rpm = cfg.idle_rpm;
    rec.max_rpm = cfg.max_rpm;
    rec.idle_us = cfg.idle_us;
    rec.max_us = cfg.max_us;
    rec.kp_us_per_rpm = cfg.kp_us_per_rpm;
    rec.ki_us_per_rpm_s = cfg.ki_us_per_rpm_s;
    rec.kd_us_per_rpm_per_s = cfg.kd_us_per_rpm_per_s;
    rec.correction_limit_us = cfg.correction_limit_us;
    rec.start_us = cfg.start_us;
    rec.start_hold_ms = cfg.start_hold_ms;
    rec.feedforward_model = cfg.feedforward_model;
    rec.hall_threshold_high_raw = hall_high;
    rec.hall_threshold_low_raw = hall_low;
    rec.board_id = custom_can_node_get_board_id();
    rec.hall_cal_min_raw = hall_cal_min;
    rec.hall_cal_max_raw = hall_cal_max;
    rec.crc32 = record_crc(&rec);

    if (!fram_write(FRAM_CONFIG_ADDR, (const uint8_t *)&rec, sizeof(rec))) {
        g_fram_present = false;
        return false;
    }

    // Verify immediately. This catches bad address width/pin/transaction issues
    // instead of printing FRAM_save=OK and then losing the board ID at reboot.
    PersistentConfigRecord verify;
    memset(&verify, 0, sizeof(verify));
    if (!fram_read(FRAM_CONFIG_ADDR, (uint8_t *)&verify, sizeof(verify))) {
        g_fram_present = false;
        return false;
    }
    if (!validate_record(&verify) ||
        verify.save_counter != rec.save_counter ||
        verify.board_id != rec.board_id ||
        verify.crc32 != rec.crc32) {
        return false;
    }

    if (board_id_is_valid(rec.board_id) && !save_board_id_record(rec.board_id)) {
        return false;
    }

    g_save_counter = rec.save_counter;
    g_loaded_board_id = rec.board_id;
    g_last_load_valid = true;
    return true;
}

bool persistent_config_save_board_id(uint32_t board_id)
{
    return save_board_id_record(board_id);
}

bool persistent_config_get_saved_board_id(uint32_t *out_board_id)
{
    if (out_board_id == NULL) {
        return false;
    }
    if (!board_id_is_valid(g_loaded_board_id)) {
        return false;
    }
    *out_board_id = g_loaded_board_id;
    return true;
}

bool persistent_config_present(void)
{
    return g_fram_present;
}

bool persistent_config_last_load_valid(void)
{
    return g_last_load_valid;
}
