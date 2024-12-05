#include <string.h>

#include "node.h"
#include "servos.h"
#include <chprintf.h>

static void makeNodeStatusMessage(
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE])
{
  memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
  int uptime_sec = TIME_I2S(chVTGetSystemTimeX());
  uint16_t vdda = 0;
  uint8_t node_mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  uint8_t node_health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;

  /*
   * Here we're using the helper for demonstrational purposes; in this simple case it could be preferred to
   * encode the values manually.
   */
  canardEncodeScalar(buffer, 0, 32, &uptime_sec);
  canardEncodeScalar(buffer, 32, 2, &node_health);
  canardEncodeScalar(buffer, 34, 3, &node_mode);
  canardEncodeScalar(buffer, 40, 16, &vdda);
}

void broadcast_node_status(struct uavcan_iface_t *iface) {
  uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
  makeNodeStatusMessage(buffer);

  static uint8_t transfer_id;

  uavcanBroadcast(iface,
      UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
      UAVCAN_PROTOCOL_NODESTATUS_ID, &transfer_id,
      CANARD_TRANSFER_PRIORITY_LOW, buffer, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
}



/*
  handle a GET_NODE_INFO request
 */
void handle_get_node_info(struct uavcan_iface_t *iface, CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt = {0};

    //pkt.status = {};
    pkt.software_version.major = SOFT_VER_MAJOR;
    pkt.software_version.minor = SOFT_VER_MINOR;
    pkt.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    pkt.software_version.vcs_commit = SOFT_HASH;

    pkt.hardware_version.major = HARD_VER_MAJOR;
    pkt.hardware_version.minor = HARD_VER_MINOR;
    memcpy(pkt.hardware_version.unique_id, (void *)UID_BASE, 12);

    uint16_t flash_size = *(const uint16_t*)FLASHSIZE_BASE;
    if(flash_size < 256U) {
      chsnprintf((char *)pkt.name.data, sizeof(pkt.name.data), "%s (WRONG FLASH %uK!!)", BOARD_NAME, flash_size);
      pkt.name.len = strlen((char *)pkt.name.data);
    } else {
      chsnprintf((char *)pkt.name.data, sizeof(pkt.name.data), "%s (%uK)", BOARD_NAME, flash_size);
      pkt.name.len = strlen((char *)pkt.name.data);
    }
   

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);
    uavcanRequestOrRespond(iface,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           total_size);
}