#ifndef NODE_H
#define NODE_H

#include "uavcan.h"

void broadcast_esc_status(struct uavcan_iface_t *iface);
void broadcast_node_status(struct uavcan_iface_t *iface);
//void handle_allocation_response(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void handle_get_node_info(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* NODE_H */