#ifndef FEETECH_SERVOS_H
#define FEETECH_SERVOS_H

#include "uavcan.h"

void feetech_servos_init(void);
void feetech_servos_disable(void);
void feetech_servos_set_failsafe(void);
void feetech_servos_request_failsafe(void);
void feetech_servos_apply_rawcommand(const struct uavcan_equipment_esc_RawCommand *msg);

#endif /* FEETECH_SERVOS_H */
