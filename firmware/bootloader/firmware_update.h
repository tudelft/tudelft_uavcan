
#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include "uavcan.h"

struct firmware_update_t {
  bool in_progress;
  char path[200];
  uint64_t file_size;
  uint64_t file_offset;
  uint8_t transfer_id;
  uint8_t node_id;
};

extern struct firmware_update_t firmware_update;
void handle_begin_frimware_update(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* FIRMWARE_UPDATE_H */