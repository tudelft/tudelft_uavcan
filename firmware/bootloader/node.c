#include <string.h>
#include <stdlib.h>

#include "node.h"
#include "firmware_update.h"


static void makeNodeStatusMessage(
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE])
{
  memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
  int uptime_sec = TIME_I2S(chVTGetSystemTimeX());
  uint16_t vdda = 0;
  uint8_t node_mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
  uint8_t node_health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  if(firmware_update.in_progress)
  {
    node_mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
  }

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
    uavcan_protocol_GetNodeInfoResponse pkt;

    //pkt.status = {};
    pkt.software_version.major = 1;
    pkt.software_version.minor = 0;

    //readUniqueID(pkt.hardware_version.unique_id);

    char name[strlen("SUPERCAN BL")+1];
    strcpy(name, "SUPERCAN BL");
    pkt.name.len = strlen("SUPERCAN BL");
    pkt.name.data = (uint8_t *)name;

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);
    uavcanRequestOrRespond(iface,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}