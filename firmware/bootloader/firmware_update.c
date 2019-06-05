#include "firmware_update.h"

struct firmware_update_t firmware_update;

/* Handle a firmware update request */
void handle_begin_frimware_update(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
  uavcan_protocol_file_BeginFirmwareUpdateResponse resp = {0};

  // Check if we are already in progress of updating the firmware
  if(firmware_update.in_progress) {
    resp.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS;
    return;
  }
  else {
    // Decode the message
    uint32_t offset = 0;
    canardDecodeScalar(transfer, 0, 8, false, (void*)&firmware_update.node_id);
    offset += 8;
    for (uint8_t i=0; i<transfer->payload_len-1; i++) {
      canardDecodeScalar(transfer, offset, 8, false, (void*)&firmware_update.path[i]);
      offset += 8;
    }

    // Reset the firmware update status
    firmware_update.transfer_id = 0;
    firmware_update.file_offset = 0;
    firmware_update.node_id = (firmware_update.node_id == 0)? transfer->source_node_id : firmware_update.node_id;
    firmware_update.in_progress = true;

    resp.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
  }

  uint8_t data[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
  uint32_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&resp, data);
  uavcanRequestOrRespond(iface,
                      transfer->source_node_id,
                      UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                      UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                      &transfer->transfer_id,
                      transfer->priority,
                      CanardResponse,
                      &data,
                      len);
}