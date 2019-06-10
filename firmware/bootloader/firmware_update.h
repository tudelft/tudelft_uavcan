
#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include "uavcan.h"

struct firmware_update_t {
  bool in_progress;
  char path[200];
  uint8_t transfer_id;
  uint8_t node_id;

  uint32_t first_word;
  uint64_t file_size;
  uint64_t file_offset;
  uint32_t last_erased_offset;
};

extern struct firmware_update_t firmware_update;
void handle_begin_frimware_update(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void handle_file_read_response(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void request_fw_file(struct uavcan_iface_t *iface);

#endif /* FIRMWARE_UPDATE_H */