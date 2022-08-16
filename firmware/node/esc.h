#ifndef ESC_H
#define ESC_H

#include "uavcan.h"

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

extern struct esc_telem_data_t esc_telem_data;
extern uint8_t esc_idx;
void esc_init(void);
void esc_disable(void);
void handle_esc_rawcommand(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void handle_tunnel_call(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* ESC_H */