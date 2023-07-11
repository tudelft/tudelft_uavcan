#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include <hal.h>

struct esc_telem_data_t {
    int8_t temp;
    float voltage;
    float current;
    uint16_t consumption;
    uint32_t erpm;
    uint16_t pole_pairs;

    // Errors
    uint32_t timeout_cnt;
};

struct esc_telem_t {
    uint8_t index;      ///< Telemetry index outputting
    uint64_t vt_delay;  ///< Frequency of the ESC can message (convert to ms)
    uint8_t type;       ///< Type of telemetry message
    UARTDriver *port;   ///< Serial port
    virtual_timer_t vt; ///< Timer for transmitting on the CAN bus

    struct esc_telem_data_t data;   ///< The received telemetry data
};
extern struct esc_telem_t esc_telem;

void esc_telem_init(void);

#endif /* ESC_TELEM_H */