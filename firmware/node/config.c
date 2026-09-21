#include "config.h"

#include <string.h>
#include "flash.h"
#include <chprintf.h>

#ifndef FEETECH_MAX_SERVOS
#define FEETECH_MAX_SERVOS 8
#endif

#if (FEETECH_MAX_SERVOS < 1) || (FEETECH_MAX_SERVOS > 8)
#error "FEETECH_MAX_SERVOS must be between 1 and 8"
#endif

#define CONFIG_ADDR_CRC     (0x0803F000)
#define CONFIG_ADDR         (CONFIG_ADDR_CRC + 0x8)


struct config_item_t config_items[] = {
/* Node configuration */
    {.name = "NODE id", .type = CONFIG_TYPE_INT, .val.i = CANARD_BROADCAST_NODE_ID, .def.i = CANARD_BROADCAST_NODE_ID, .min.i = 0, .max.i = CANARD_MAX_NODE_ID},
    {.name = "CAN termination", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3},
    {.name = "CAN bridge", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 2},

/* Servos configuration */
    {.name = "SERVO failsafe timeout (ms)", .type = CONFIG_TYPE_INT, .val.i = 100, .def.i = 100, .min.i = 0, .max.i = 20000},
#ifdef SERVO1_LINE
    {.name = "SERVO1 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO1 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1000, .def.i = 1000, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO2_LINE
    {.name = "SERVO2 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO2 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO3_LINE
    {.name = "SERVO3 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO3 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO4_LINE
    {.name = "SERVO4 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO4 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO5_LINE
    {.name = "SERVO5 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO5 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO6_LINE
    {.name = "SERVO6 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO6 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO7_LINE
    {.name = "SERVO7 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO7 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO8_LINE
    {.name = "SERVO8 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO8 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO9_LINE
    {.name = "SERVO9 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO9 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif
#ifdef SERVO10_LINE
    {.name = "SERVO10 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "SERVO10 failsafe", .type = CONFIG_TYPE_INT, .val.i = 1500, .def.i = 1500, .min.i = 0, .max.i = 2400},
#endif

/* Feetech STS UART servo configuration */
    {.name = "FT baud", .type = CONFIG_TYPE_INT, .val.i = 1000000, .def.i = 1000000, .min.i = 38400, .max.i = 1000000},
    {.name = "FT telem position frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 10, .def.f = 10, .min.f = 0.0f, .max.f = 1000.0f},
    {.name = "FT telem speed enable", .type = CONFIG_TYPE_BOOL, .val.b = 1, .def.b = 1, .min.i = 0, .max.i = 1},
    {.name = "FT telem load enable", .type = CONFIG_TYPE_BOOL, .val.b = 1, .def.b = 1, .min.i = 0, .max.i = 1},
    {.name = "FT telem current/temp frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0.0f, .max.f = 1000.0f},
#if FEETECH_MAX_SERVOS >= 1
    {.name = "FT1 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT1 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT1 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 2
    {.name = "FT2 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT2 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT2 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 3
    {.name = "FT3 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT3 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT3 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 4
    {.name = "FT4 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT4 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT4 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 5
    {.name = "FT5 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT5 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT5 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 6
    {.name = "FT6 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT6 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT6 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 7
    {.name = "FT7 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT7 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT7 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif
#if FEETECH_MAX_SERVOS >= 8
    {.name = "FT8 bus+id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3253},
    {.name = "FT8 index", .type = CONFIG_TYPE_INT, .val.i = 255, .def.i = 255, .min.i = 0, .max.i = 255},
    {.name = "FT8 failsafe", .type = CONFIG_TYPE_INT, .val.i = 2048, .def.i = 2048, .min.i = 0, .max.i = 4095},
#endif

#if USE_FAULHABER_CTRL
/* FAULHABER configuration */
    {.name = "FAULHABER index", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 255},
    {.name = "FAULHABER port", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3},
    {.name = "FAULHABER baudrate", .type = CONFIG_TYPE_INT, .val.i = 115200, .def.i = 115200, .min.i = 4800, .max.i = 1000000},
    {.name = "FAULHABER telem frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 10, .def.f = 10, .min.f = 0.0001, .max.f = 10000},
    {.name = "FAULHABER node number", .type = CONFIG_TYPE_INT, .val.i = 1, .def.i = 1, .min.i = 0, .max.i = 255},
    {.name = "FAULHABER start timeout (s)", .type = CONFIG_TYPE_INT, .val.i = 5, .def.i = 5, .min.i = 0, .max.i = 1000000},
    {.name = "FAULHABER home method", .type = CONFIG_TYPE_INT, .val.i = 19, .def.i = 19, .min.i = 0, .max.i = 255},
    {.name = "FAULHABER deadband", .type = CONFIG_TYPE_INT, .val.i = 100, .def.i = 100, .min.i = 0, .max.i = INT32_MAX},
    {.name = "FAULHABER min position", .type = CONFIG_TYPE_INT, .val.i = 60000, .def.i = 60000, .min.i = 0, .max.i = INT32_MAX},
    {.name = "FAULHABER max position", .type = CONFIG_TYPE_INT, .val.i = 3450000, .def.i = 3450000, .min.i = 0, .max.i = INT32_MAX},
#endif

#if USE_ESC_TELEM
/* ESC telemetry configuration */
    {.name = "ESC telem index", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 255},
    {.name = "ESC telem frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 10, .def.f = 10, .min.f = 0.0001, .max.f = 10000},
    {.name = "ESC telem type", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 2},
    {.name = "ESC telem port", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3},
    {.name = "ESC telem pole pairs", .type = CONFIG_TYPE_INT, .val.i = 1, .def.i = 1, .min.i = 1, .max.i = 1000},
#endif

#if USE_IE_FUELCELL
/* IE Fuelcell configuration */
    {.name = "IE FC frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 10, .def.f = 10, .min.f = 0.0001, .max.f = 10000},
    {.name = "IE FC port", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3},
    {.name = "IE FC baud", .type = CONFIG_TYPE_INT, .val.i = 9600, .def.i = 9600, .min.i = 4800, .max.i = 1000000},
#endif

#if USE_DRS_PARACHUTE
/* DRS configuration */
    {.name = "DRS index", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 255},
    {.name = "DRS port", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3},
    {.name = "DRS baudrate", .type = CONFIG_TYPE_INT, .val.i = 115200, .def.i = 115200, .min.i = 4800, .max.i = 1000000},
#endif

#if USE_TFMINI
/* TFMini configuration */
    {.name = "TFMINI port", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 3},
    {.name = "TFMINI baudrate", .type = CONFIG_TYPE_INT, .val.i = 115200, .def.i = 115200, .min.i = 4800, .max.i = 1000000},
    {.name = "TFMINI frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0.0001, .max.f = 10000},
#endif

#if USE_ADCS
/* POWER ADC's configuration */
    {.name = "POWER1 device id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 65536},
    {.name = "POWER1 frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 10000},
    {.name = "POWER1 volt chan", .type = CONFIG_TYPE_INT, .val.i = ADC_POWER1_CHANNEL, .def.i = ADC_POWER1_CHANNEL, .min.i = -1, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "POWER1 current chan", .type = CONFIG_TYPE_INT, .val.i = -1, .def.i = -1, .min.i = -1, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "POWER1 volt mult", .type = CONFIG_TYPE_FLOAT, .val.f = ADC_POWER1_MUL, .def.f = ADC_POWER1_MUL, .min.f = 0, .max.f = 1000000},
    {.name = "POWER1 volt offset", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 1000000},
    {.name = "POWER1 current mult", .type = CONFIG_TYPE_FLOAT, .val.f = 50.0, .def.f = 50.0, .min.f = 0, .max.f = 1000000},
    {.name = "POWER1 current offset", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 1000000},

    {.name = "POWER2 device id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 65536},
    {.name = "POWER2 frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 10000},
    {.name = "POWER2 volt chan", .type = CONFIG_TYPE_INT, .val.i = ADC_POWER2_CHANNEL, .def.i = ADC_POWER2_CHANNEL, .min.i = -1, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "POWER2 current chan", .type = CONFIG_TYPE_INT, .val.i = -1, .def.i = -1, .min.i = -1, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "POWER2 volt mult", .type = CONFIG_TYPE_FLOAT, .val.f = ADC_POWER2_MUL, .def.f = ADC_POWER2_MUL, .min.f = 0, .max.f = 1000000},
    {.name = "POWER2 volt offset", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 1000000},
    {.name = "POWER2 current mult", .type = CONFIG_TYPE_FLOAT, .val.f = 50.0, .def.f = 50.0, .min.f = 0, .max.f = 1000000},
    {.name = "POWER2 current offset", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 1000000},

/* NTC ADC's configuration */
    {.name = "NTC1 device id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 65536},
    {.name = "NTC1 channel", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "NTC1 frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 10000},
    {.name = "NTC1 pull up R", .type = CONFIG_TYPE_FLOAT, .val.f = 10000, .def.f = 10000, .min.f = 0, .max.f = 1000000},
    {.name = "NTC1 SH eq a", .type = CONFIG_TYPE_FLOAT, .val.f = 0.00103753243, .def.f = 0.00103753243, .min.f = 0, .max.f = 1000000},
    {.name = "NTC1 SH eq b", .type = CONFIG_TYPE_FLOAT, .val.f = 0.00025150905, .def.f = 0.00025150905, .min.f = 0, .max.f = 1000000},
    {.name = "NTC1 SH eq c", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 1000000},

    {.name = "NTC2 device id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 65536},
    {.name = "NTC2 channel", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "NTC2 frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 10000},
    {.name = "NTC2 pull up R", .type = CONFIG_TYPE_FLOAT, .val.f = 10000, .def.f = 10000, .min.f = 0, .max.f = 1000000},
    {.name = "NTC2 SH eq a", .type = CONFIG_TYPE_FLOAT, .val.f = 0.00103753243, .def.f = 0.00103753243, .min.f = 0, .max.f = 1000000},
    {.name = "NTC2 SH eq b", .type = CONFIG_TYPE_FLOAT, .val.f = 0.00025150905, .def.f = 0.00025150905, .min.f = 0, .max.f = 1000000},
    {.name = "NTC2 SH eq c", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 1000000},

/* POTMETER ADC's configuration */
    {.name = "POTMETER1 device id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 65536},
    {.name = "POTMETER1 channel", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "POTMETER1 frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 10000},
    {.name = "POTMETER1 type", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 5},
    {.name = "POTMETER1 cal_a", .type = CONFIG_TYPE_FLOAT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 1000000},
    {.name = "POTMETER1 cal_b", .type = CONFIG_TYPE_FLOAT, .val.i = 1, .def.i = 1, .min.i = 0, .max.i = 1000000},

    {.name = "POTMETER2 device id", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 65536},
    {.name = "POTMETER2 channel", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = ADC_MAX_CHANNELS-1},
    {.name = "POTMETER2 frequency", .type = CONFIG_TYPE_FLOAT, .val.f = 0, .def.f = 0, .min.f = 0, .max.f = 10000},
    {.name = "POTMETER2 type", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 5},
    {.name = "POTMETER2 cal_a", .type = CONFIG_TYPE_FLOAT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 1000000},
    {.name = "POTMETER2 cal_b", .type = CONFIG_TYPE_FLOAT, .val.i = 1, .def.i = 1, .min.i = 0, .max.i = 1000000},
#endif
};
uint16_t config_crc = 0;
uint16_t config_crc_r1 = 0;
uint16_t config_crc_r2 = 0;
const uint8_t config_items_cnt = sizeof(config_items) / sizeof(struct config_item_t);

//char (*__kaboom)[sizeof(config_items)] = 1; // Way to verify the size while compiling

static uint16_t calc_crc(struct config_item_t *config_items, uint8_t len) {
    uint8_t *data = (uint8_t *)config_items;
    uint32_t size = sizeof(struct config_item_t) * len;
    uint16_t crc = 0xFFFF;

    for(uint32_t i = 0; i < size; i++) {
        crc = ((data[i] << 8) ^ crc);

        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x8000)
                crc = (crc << 1) ^ 0x8005;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

void config_init(void) {
    config_crc = calc_crc(config_items, config_items_cnt);
    config_crc_r1 = *(uint16_t *)(CONFIG_ADDR_CRC);
    config_crc_r2 = *(uint16_t *)(CONFIG_ADDR_CRC+2);

    if(config_crc_r1 != config_crc || config_crc_r2 != config_crc) {
        config_reset();
        config_save();
    }
    else {
        config_read();
    }
}

void config_save(void) {
    union config_val_t values[config_items_cnt];
    uint32_t size = sizeof(union config_val_t) * config_items_cnt;

    // Copy the values for saving
    for(uint8_t i = 0; i < config_items_cnt; i++)
        values[i] = config_items[i].val;

    flash_erase_pages(CONFIG_ADDR_CRC, 4 + size);
    flash_write_block(CONFIG_ADDR, (uint8_t *)values, size);

    if(flash_verify_block((void*)CONFIG_ADDR, (uint8_t *)values, size) == 1) {
        flash_write_block(CONFIG_ADDR_CRC, (uint8_t *)&config_crc, 2);
        flash_write_block(CONFIG_ADDR_CRC+2, (uint8_t *)&config_crc, 2);
    }
}

void config_read(void) {
    volatile union config_val_t *values = (volatile union config_val_t *)CONFIG_ADDR;

    // Copy the values for loading
    for(uint8_t i = 0; i < config_items_cnt; i++)
        config_items[i].val = values[i];
}

void config_reset(void) {
    // Reset to default values
    for(uint8_t i = 0; i < config_items_cnt; i++)
        config_items[i].val = config_items[i].def;
}

/* Get a config item by name */
struct config_item_t *config_get_by_name(char* name, uint8_t len) {
    if(len == 0)
        len = strlen(name);
    
    // Search the config
    for(uint8_t i = 0; i < config_items_cnt; i++) {
        if(strncmp(config_items[i].name, name, len) == 0)
            return &config_items[i];
    }

    return NULL;
}

int16_t config_get_i16(const char *name, int16_t default_value) {
    struct config_item_t *item = config_get_by_name((char *)name, 0);

    if (item == NULL) {
        return default_value;
    }

    return (int16_t)item->val.i;
}

uint8_t config_get_u8(const char *name, uint8_t default_value) {
    struct config_item_t *item = config_get_by_name((char *)name, 0);

    if (item == NULL) {
        return default_value;
    }

    return (uint8_t)item->val.i;
}

uint16_t config_get_u16(const char *name, uint16_t default_value) {
    struct config_item_t *item = config_get_by_name((char *)name, 0);

    if (item == NULL) {
        return default_value;
    }

    return (uint16_t)item->val.i;
}

uint32_t config_get_u32(const char *name, uint32_t default_value) {
    struct config_item_t *item = config_get_by_name((char *)name, 0);

    if (item == NULL) {
        return default_value;
    }

    return (uint32_t)item->val.i;
}

float config_get_f32(const char *name, float default_value) {
    struct config_item_t *item = config_get_by_name((char *)name, 0);

    if (item == NULL) {
        return default_value;
    }

    return item->val.f;
}

bool config_get_bool(const char *name, bool default_value) {
    struct config_item_t *item = config_get_by_name((char *)name, 0);

    if (item == NULL) {
        return default_value;
    }

    if (item->type == CONFIG_TYPE_BOOL) {
        return item->val.b != 0U;
    }

    if (item->type == CONFIG_TYPE_INT) {
        return item->val.i != 0;
    }

    return default_value;
}

static void config_set_resp(struct config_item_t *item, struct uavcan_protocol_param_GetSetResponse *resp) {
    memcpy(resp->name.data, item->name, strlen(item->name));
    resp->name.len = strlen(item->name);

    switch(item->type) {
        case CONFIG_TYPE_EMPTY:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
            break;
        case CONFIG_TYPE_INT:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            resp->value.integer_value = item->val.i;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            resp->default_value.integer_value = item->def.i;
            resp->min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            resp->min_value.integer_value = item->min.i;
            resp->max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            resp->max_value.integer_value = item->max.i;
            break;
        case CONFIG_TYPE_FLOAT:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            resp->value.real_value = item->val.f;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            resp->default_value.real_value = item->def.f;
            resp->min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            resp->min_value.real_value = item->min.f;
            resp->max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            resp->max_value.real_value = item->max.f;
            break;
        case CONFIG_TYPE_BOOL:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
            resp->value.boolean_value = item->val.b;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
            resp->default_value.boolean_value = item->def.b;
            break;
        case CONFIG_TYPE_STRING:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
            memcpy(resp->value.string_value.data, item->val.s, strlen(item->val.s));
            resp->value.string_value.len = (uint8_t)strlen(item->val.s);
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
            memcpy(resp->default_value.string_value.data, item->def.s, strlen(item->def.s));
            resp->default_value.string_value.len = (uint8_t)strlen(item->def.s);
            break;
    }
}

/* Disable enum comparison and conversion */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wenum-compare"
#pragma GCC diagnostic ignored "-Wenum-conversion"

/*
  handle a PARAM_GETSET request
 */
void handle_param_getset(struct uavcan_iface_t *iface, CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_param_GetSetResponse resp = {0};
    struct uavcan_protocol_param_GetSetRequest req = {0};

    // Decode the incoming request
	if(uavcan_protocol_param_GetSetRequest_decode(transfer, &req))
        return;
    
    // Get the correct config item
    struct config_item_t *item = NULL;
    if(req.name.data && req.name.len) {
        item = config_get_by_name((char *)req.name.data, req.name.len);
    } else if (req.index < config_items_cnt) {
        item = &config_items[req.index];
    }

    // If we did find the config item
    if(item != NULL) {
        // Apply requested value when incoming type matches the config item type.
        switch(item->type) {
            case CONFIG_TYPE_INT:
                if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                    item->val.i = req.value.integer_value;
                }
                break;
            case CONFIG_TYPE_FLOAT:
                if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
                    item->val.f = req.value.real_value;
                }
                break;
            case CONFIG_TYPE_BOOL:
                if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE) {
                    item->val.b = req.value.boolean_value;
                }
                break;
            default:
                break;
        }

        // Set the response
        config_set_resp(item, &resp);
    }
    // The config item was not found
    else {
        resp.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        memset(resp.name.data, 0, 1);
        resp.name.len = 0;
    }

    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&resp, buffer);
    uavcanRequestOrRespond(iface,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           total_size);

    if((!req.name.data || !req.name.len) && req.index == 0) {
        /* Debug information */
        char msg[90];
        chsnprintf(msg, 90, "Reading start (f1:%X f2:%X c:%X)", config_crc_r1, config_crc_r2, config_crc);
        uavcanDebug(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "Config", msg);
    }
    else if(item == NULL) {
        char msg[90];
        chsnprintf(msg, 90, "Reading finished (len: %d)", config_items_cnt);
        uavcanDebug(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "Config", msg);
    }
}

#pragma GCC diagnostic pop

void handle_param_execute_opcode(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_param_ExecuteOpcodeResponse resp = {0};
    struct uavcan_protocol_param_ExecuteOpcodeRequest req = {0};

    if(uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req))
        return;

    resp.ok = false;
    if(req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        config_reset();
        config_save();
        resp.ok = true;
    } else if(req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        config_save();
        resp.ok = true;
    }

    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&resp, buffer);
    uavcanRequestOrRespond(iface,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           total_size);

    if(req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        uavcanDebug(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "Config", "Erased");
    } else if(req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        uavcanDebug(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "Config", "Saved");
    }
}