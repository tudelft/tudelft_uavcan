#ifndef SERVOS_H
#define SERVOS_H

#include "uavcan.h"

void servos_init(void);
void servos_disable(void);
void handle_esc_rawcommand(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* SERVOS_H */