#ifndef UAVCAN_H
#define UAVCAN_H

#include <ch.h>
#include <hal.h>
#include <canard.h>

// UAVCAN messages
#include <uavcan/protocol/GetNodeInfo.h>
#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/RestartNode.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>
#include <uavcan/protocol/debug/LogMessage.h>
#include <uavcan/protocol/file/BeginFirmwareUpdate.h>
#include <uavcan/protocol/file/Read.h>
#include <uavcan/equipment/esc/RawCommand.h>
#include <uavcan/equipment/esc/Status.h>
#include <uavcan/equipment/device/Temperature.h>
#include <uavcan/equipment/actuator/Status.h>
#include <uavcan/protocol/param/GetSet.h>
#include <uavcan/protocol/param/ExecuteOpcode.h>
#include <uavcan/protocol/param/ExecuteOpcode.h>
#include <uavcan/tunnel/Call.h>


struct uavcan_iface_t {
  CANDriver *can_driver;
  uint32_t can_baudrate;
  CANConfig can_cfg;

  event_source_t tx_request;
  mutex_t mutex;
  void *thread_rx_wa;
  void *thread_tx_wa;
  void *thread_uavcan_wa;
  size_t thread_rx_wa_size;
  size_t thread_tx_wa_size;
  size_t thread_uavcan_wa_size;

  uint8_t node_id;
  CanardInstance canard;
  uint8_t canard_memory_pool[1024];

  // Dynamic node id allocation
  uint32_t send_next_node_id_allocation_request_at_ms;
  uint8_t node_id_allocation_unique_id_offset;

  // Errors
  uint32_t transmit_err_cnt;
  uint32_t transmit_err_flush_cnt;
};

void uavcanInit(void);

int uavcanRequestOrRespond(struct uavcan_iface_t *iface, uint8_t destination_node_id, uint64_t data_type_signature,
    uint8_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, CanardRequestResponse kind,
    const void* payload, uint16_t payload_len);

int uavcanBroadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id,
    uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);

int uavcanBroadcastAll(uint64_t data_type_signature, uint16_t data_type_id,
    uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);

#endif /* UAVCAN_H */