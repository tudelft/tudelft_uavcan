#ifndef FAULHABER_CTRL_H
#define FAULHABER_CTRL_H

#include <hal.h>

struct faulhaber_ctrl_t {
    uint8_t index;              ///< Telemetry and actuator index
    UARTDriver *port;           ///< Serial port
    uint64_t telem_vt_delay;    ///< Frequency of the telemetry can message (convert to ms)
    UARTConfig uart_cfg;        ///< UART configuration

    uint8_t node_nb;            ///< Faulhaber node number to actuate
    uint32_t start_timeout_s;   ///< Start timeout in seconds
    uint8_t home_method;        ///< Faulhaber homing method (see drive functions manual)
    uint32_t deadband;          ///< Deadband in which the output stage is turned off
    uint32_t min_pos;           ///< Minimum position
    uint32_t max_pos;           ///< Maximum position

    bool homing_completed;      ///< Once the homing is completed
    bool position_ready;        ///< Ready for receiving positions
    bool target_reached;        ///< When the target position is reached
    int32_t actual_position;    ///< Actual measured position
    int32_t target_position;    ///< The target position
};
extern struct faulhaber_ctrl_t faulhaber_ctrl;

void faulhaber_ctrl_init(void);

#endif /* FAULHABER_CTRL_H */