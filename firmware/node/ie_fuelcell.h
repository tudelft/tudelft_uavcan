#ifndef IE_FUELCELL_H
#define IE_FUELCELL_H

#include <hal.h>

struct ie_fuelcell_data_t {
    uint8_t tank_pressure;        // percentage
    uint8_t regulated_pressure;   // *100

    uint16_t battery_voltage;     // decivolt *10
    uint16_t output_power;
    uint16_t spm_power;
    int16_t battery_power;
    uint8_t psu_state;
    uint8_t error_code;
    uint8_t sub_code;
    uint8_t checksum;

    // Other things
    char term[128];
    uint8_t term_id;
    uint8_t term_size;
    uint8_t calc_checksum;
    bool in_string;
    bool received;
    uint32_t timeout_cnt;
};

struct ie_fuelcell_t {
    uint64_t vt_delay;      ///< Frequency of the IE Fuelcell can message (convert to ms)
    UARTDriver *port;       ///< Serial port
    virtual_timer_t vt;     ///< Timer for transmitting on the CAN bus

    struct ie_fuelcell_data_t data;   ///< The received telemetry data
};
extern struct ie_fuelcell_t ie_fuelcell;

void ie_fuelcell_init(void);

#endif /* IE_FUELCELL_H */