#ifndef ESC_H
#define ESC_H

#include "uavcan.h"

void esc_init(void);
void handle_esc_rawcommand(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* ESC_H */