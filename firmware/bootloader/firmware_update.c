#include "firmware_update.h"
#include "flash.h"

#include <string.h>

struct firmware_update_t firmware_update;
#define APP_START_ADDRESS     0x08008000
#define APP_MAX_SIZE          0x30000

/* Handle a firmware update request */
void handle_begin_firmware_update(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
  uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
  struct uavcan_protocol_file_BeginFirmwareUpdateResponse resp = {0};
  resp.error = 0;
  resp.optional_error_message.len = 0;

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

    // Remove previous firmware
    flash_erase_pages(APP_START_ADDRESS, APP_MAX_SIZE);

    // Reset the firmware update status
    firmware_update.iface = iface;
    firmware_update.transfer_id = 0;
    firmware_update.file_offset = 0;
    firmware_update.last_erased_offset = 0;
    firmware_update.node_id = (firmware_update.node_id == 0)? transfer->source_node_id : firmware_update.node_id;
    firmware_update.in_progress = true;

    resp.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
  }

  uint32_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&resp, buffer);
  uavcanRequestOrRespond(iface,
                      transfer->source_node_id,
                      UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                      UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                      &transfer->transfer_id,
                      transfer->priority,
                      CanardResponse,
                      &buffer,
                      len);
}

void request_fw_file(struct uavcan_iface_t *iface) {
  uint8_t buffer[strlen(firmware_update.path)+8];

  // Sanity check
  if(!firmware_update.in_progress)
    return;

  canardEncodeScalar(buffer, 0, 40, &firmware_update.file_offset);
  uint32_t offset = 40;
  uint8_t len = strlen(firmware_update.path);
  for (uint8_t i=0; i<len; i++) {
      canardEncodeScalar(buffer, offset, 8, &firmware_update.path[i]);
      offset += 8;
  }

  uint32_t total_size = (offset+7)/8;
  uavcanRequestOrRespond(iface,
                      127,
                      UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                      UAVCAN_PROTOCOL_FILE_READ_ID,
                      &firmware_update.transfer_id,
                      CANARD_TRANSFER_PRIORITY_LOW,
                      CanardRequest,
                      &buffer[0],
                      total_size);
}

/*
  handle response to file read for fw update
 */
extern void jump_to_app(void);
void handle_file_read_response(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
    if ((transfer->transfer_id+1)%256 != firmware_update.transfer_id ||
        transfer->source_node_id != firmware_update.node_id) {
        return;
    }
    int16_t error = 0;
    canardDecodeScalar(transfer, 0, 16, true, (void*)&error);
    volatile uint16_t len = transfer->payload_len - 2;

    uint32_t offset = 16;
    uint32_t buf32[(len+3)/4];
    uint8_t *buf = (uint8_t *)&buf32[0];
    for (uint16_t i=0; i<len; i++) {
      canardDecodeScalar(transfer, offset, 8, false, (void*)&buf[i]);
      offset += 8;
    }

    // First erase pages if needed
    if(firmware_update.last_erased_offset < firmware_update.file_offset)
      firmware_update.last_erased_offset = firmware_update.file_offset;

    //int32_t erase_size = firmware_update.file_offset + len - firmware_update.last_erased_offset;
    //if(erase_size > 0)
    //  firmware_update.last_erased_offset = flash_erase_pages((APP_START_ADDRESS+firmware_update.last_erased_offset+1), erase_size) - APP_START_ADDRESS - 4;
    

    // Flash the data (except the first word)
    if(firmware_update.file_offset == 0) {
      firmware_update.first_word = buf32[0];
      flash_write_block((void *)(APP_START_ADDRESS + 0x4), &buf[4], len-4);
    }
    else
    {
      uint32_t offs = firmware_update.file_offset;
      flash_write_block((void *)(APP_START_ADDRESS + offs), buf, len);
    }

    firmware_update.file_offset += len;
    
    if (len < 256) {
      firmware_update.in_progress = false;
      firmware_update.node_id = 0;
      // now flash the first word
      flash_write_block((void *)APP_START_ADDRESS, (uint8_t *)&firmware_update.first_word, 4);
      jump_to_app();
    }
}