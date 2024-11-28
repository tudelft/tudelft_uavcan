#include "firmware_update.h"
#include "flash.h"
#include "chprintf.h"

#include <string.h>

struct firmware_update_t firmware_update;
#define APP_START_ADDRESS     0x08008000
#define BOOTLOADER_SIZE       0x8000

/* Handle a firmware update request */
void handle_begin_firmware_update(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
  uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
  struct uavcan_protocol_file_BeginFirmwareUpdateResponse resp = {0};
  resp.error = 0;
  resp.optional_error_message.len = 0;

  // Check if we are already in progress of updating the firmware
  bool in_progress = false;
  if(firmware_update.in_progress) {
    resp.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS;
    in_progress = true;
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
    firmware_update.iface = iface;
    firmware_update.transfer_id = 0;
    firmware_update.file_offset = 0;
    firmware_update.last_erased_offset = 0;
    firmware_update.node_id = (firmware_update.node_id == 0)? transfer->source_node_id : firmware_update.node_id;

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

  // Request file info if not in progress
  if(in_progress)
    return;

  // Request file info 
  struct uavcan_protocol_file_GetInfoRequest req = {0};
  req.path.path.len = strlen(firmware_update.path);
  for (uint8_t i=0; i<req.path.path.len; i++) {
    req.path.path.data[i] = firmware_update.path[i];
  }

  uint8_t buffer2[UAVCAN_PROTOCOL_FILE_GETINFO_REQUEST_MAX_SIZE];
  len = uavcan_protocol_file_GetInfoRequest_encode(&req, buffer2);
  uavcanRequestOrRespond(iface,
                      firmware_update.node_id,
                      UAVCAN_PROTOCOL_FILE_GETINFO_SIGNATURE,
                      UAVCAN_PROTOCOL_FILE_GETINFO_ID,
                      &firmware_update.transfer_id,
                      CANARD_TRANSFER_PRIORITY_LOW,
                      CanardRequest,
                      &buffer2,
                      len);
}

void handle_file_getinfo_response(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
  if ((transfer->transfer_id+1)%256 != firmware_update.transfer_id ||
      transfer->source_node_id != firmware_update.node_id) {
      return;
  }

  uint64_t size = 0;
  uint16_t flash_size = *(const uint16_t*)FLASHSIZE_BASE - (BOOTLOADER_SIZE/1024);
  canardDecodeScalar(transfer, 0, 40, true, (void*)&size);

  // Remove previous firmware
  flash_erase_pages(APP_START_ADDRESS, (uint64_t)size);
  firmware_update.file_size = size;
  firmware_update.in_progress = true;

  struct uavcan_protocol_debug_LogMessage logMsg;
  logMsg.level.value = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG;
  strcpy((char*)logMsg.source.data, "Firmware Update");
  logMsg.source.len = strlen((char*)logMsg.source.data);

  if(flash_size*1024 < firmware_update.file_size) {
    logMsg.level.value = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR;
    chsnprintf((char *)logMsg.text.data, 90, "Not enough space / erased (%uK < %uK)", flash_size, (uint16_t)(firmware_update.file_size/1024));
  } else {
    chsnprintf((char *)logMsg.text.data, 90, "Erased %uK out of %uK", (uint16_t)(firmware_update.file_size/1024), flash_size);
  }
  logMsg.text.len = strlen((char*)logMsg.text.data);

  uint8_t send_buf[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
  uint32_t len = uavcan_protocol_debug_LogMessage_encode(&logMsg, send_buf);

  uint8_t debug_transfer_id = 124;
  uavcanBroadcast(iface,
                  UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                  UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                  &debug_transfer_id,
                  CANARD_TRANSFER_PRIORITY_LOW,
                  send_buf,
                  len);
}

static void sendProgress(struct uavcan_iface_t *iface, uint8_t progress) {
  struct uavcan_protocol_debug_LogMessage logMsg;
  logMsg.level.value = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO;
  strcpy((char*)logMsg.source.data, "Firmware Update");
  logMsg.source.len = strlen((char*)logMsg.source.data);

  chsnprintf((char *)logMsg.text.data, 90, "Progress: %u%%", progress);
  logMsg.text.len = strlen((char*)logMsg.text.data);

  uint8_t send_buf[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
  uint32_t len = uavcan_protocol_debug_LogMessage_encode(&logMsg, send_buf);

  uint8_t debug_transfer_id = 125;
  uavcanBroadcast(iface,
                  UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                  UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                  &debug_transfer_id,
                  CANARD_TRANSFER_PRIORITY_LOW,
                  send_buf,
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
                      firmware_update.node_id,
                      UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                      UAVCAN_PROTOCOL_FILE_READ_ID,
                      &firmware_update.transfer_id,
                      CANARD_TRANSFER_PRIORITY_LOW,
                      CanardRequest,
                      &buffer[0],
                      total_size);

  static uint8_t lastProgress = 0;
  uint8_t progress = (uint8_t)(100*firmware_update.file_offset/firmware_update.file_size);
  if(progress-lastProgress >= 5 || progress < lastProgress) {
    sendProgress(iface, progress);
    lastProgress = progress;
  }
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