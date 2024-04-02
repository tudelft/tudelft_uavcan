#include <string.h>
#include <stdlib.h>

#include "uavcan.h"
#include "config.h"
#include "node.h"
#include "servos.h"

#if STM32_CAN_USE_CAN1
static THD_WORKING_AREA(can1_rx_wa, 1024*4);
static THD_WORKING_AREA(can1_tx_wa, 1024*4);
static THD_WORKING_AREA(can1_uavcan_wa, 2048*4);

static struct uavcan_iface_t can1_iface = {
  .can_driver = &CAND1,
  .can_baudrate = 1000000,
  .can_cfg = { 0 },
  .thread_rx_wa = can1_rx_wa,
  .thread_rx_wa_size = sizeof(can1_rx_wa),
  .thread_tx_wa = can1_tx_wa,
  .thread_tx_wa_size = sizeof(can1_tx_wa),
  .thread_uavcan_wa = can1_uavcan_wa,
  .thread_uavcan_wa_size = sizeof(can1_uavcan_wa),
  .node_id = CANARD_BROADCAST_NODE_ID,
  .transmit_err_cnt = 0,
  .transmit_err_flush_cnt = 0
};
#endif

#if STM32_CAN_USE_CAN2
static THD_WORKING_AREA(can2_rx_wa, 1024*4);
static THD_WORKING_AREA(can2_tx_wa, 1024*4);
static THD_WORKING_AREA(can2_uavcan_wa, 2048*4);

static struct uavcan_iface_t can2_iface = {
  .can_driver = &CAND2,
  .can_baudrate = 1000000,
  .can_cfg = { 0 },
  .thread_rx_wa = can2_rx_wa,
  .thread_rx_wa_size = sizeof(can2_rx_wa),
  .thread_tx_wa = can2_tx_wa,
  .thread_tx_wa_size = sizeof(can2_tx_wa),
  .thread_uavcan_wa = can2_uavcan_wa,
  .thread_uavcan_wa_size = sizeof(can2_uavcan_wa),
  .node_id = CANARD_BROADCAST_NODE_ID,
  .transmit_err_cnt = 0,
  .transmit_err_flush_cnt = 0
};
#endif

/**
 * Different briding modes for UAVCAN
*/
enum uavcan_bridge_t {
  UAVCAN_BRIDGE_NONE = 0,   ///< No bridge
  UAVCAN_BRIDGE_CAN1TOCAN2, ///< Bridge from CAN1 to CAN2
  UAVCAN_BRIDGE_CAN2TOCAN1  ///< Bridge from CAN2 to CAN1
};

static enum uavcan_bridge_t uavcan_bridge = UAVCAN_BRIDGE_NONE;

/**
 * Blocking version of the Request or Respons canard sending
 * This will also trigger a Transmit request for the destination interface
 */
int uavcanRequestOrRespond(struct uavcan_iface_t *iface,    ///< The interface to write to
                           uint8_t destination_node_id,     ///< Node ID of the server/client
                           uint64_t data_type_signature,    ///< See above
                           uint8_t data_type_id,            ///< Refer to the specification
                           uint8_t* inout_transfer_id,      ///< Pointer to a persistent variable with the transfer ID
                           uint8_t priority,                ///< Refer to definitions CANARD_TRANSFER_PRIORITY_*
                           CanardRequestResponse kind,      ///< Refer to CanardRequestResponse
                           const void* payload,             ///< Transfer payload
                           uint16_t payload_len)            ///< Length of the above, in bytes
{
  int res;
  chMtxLock(&iface->mutex);
  res = canardRequestOrRespond(&iface->canard, destination_node_id, data_type_signature, data_type_id, inout_transfer_id,
      priority, kind, payload, payload_len);
  chMtxUnlock(&iface->mutex);
  chEvtBroadcast(&iface->tx_request);
  return res;
}

/**
 * Blocking version of the Broadcast canard sending
 * This will also trigger a Transmit request for the destination interface
 */
int uavcanBroadcast(struct uavcan_iface_t *iface,   ///< The interface to write to
                    uint64_t data_type_signature,   ///< See above
                    uint16_t data_type_id,          ///< Refer to the specification
                    uint8_t* inout_transfer_id,     ///< Pointer to a persistent variable containing the transfer ID
                    uint8_t priority,               ///< Refer to definitions CANARD_TRANSFER_PRIORITY_*
                    const void* payload,            ///< Transfer payload
                    uint16_t payload_len)           ///< Length of the above, in bytes
{
  int res;
  chMtxLock(&iface->mutex);
  res = canardBroadcast(&iface->canard, data_type_signature, data_type_id, inout_transfer_id, priority, payload, payload_len);
  chMtxUnlock(&iface->mutex);
  chEvtBroadcast(&iface->tx_request);
  return res;
}

/**
 * Broadcast on all ifaces
*/
int uavcanBroadcastAll(uint64_t data_type_signature,   ///< See above
                    uint16_t data_type_id,          ///< Refer to the specification
                    uint8_t* inout_transfer_id,     ///< Pointer to a persistent variable containing the transfer ID
                    uint8_t priority,               ///< Refer to definitions CANARD_TRANSFER_PRIORITY_*
                    const void* payload,            ///< Transfer payload
                    uint16_t payload_len)           ///< Length of the above, in bytes
{
  int res;
#ifdef STM32_CAN_USE_CAN1
  chMtxLock(&can1_iface.mutex);
  res = canardBroadcast(&can1_iface.canard, data_type_signature, data_type_id, inout_transfer_id, priority, payload, payload_len);
  chMtxUnlock(&can1_iface.mutex);
  chEvtBroadcast(&can1_iface.tx_request);
#endif
#ifdef STM32_CAN_USE_CAN2
  chMtxLock(&can2_iface.mutex);
  res = canardBroadcast(&can2_iface.canard, data_type_signature, data_type_id, inout_transfer_id, priority, payload, payload_len);
  chMtxUnlock(&can2_iface.mutex);
  chEvtBroadcast(&can2_iface.tx_request);
#endif
  return res;
}

// void uavcanLog(struct uavcan_iface_t *iface, char *msg, uint8_t len) {
//   // char *test = "SUPERCAN";
//   // uavcan_protocol_debug_LogMessage logMsg;
//   // logMsg.level.value = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG;
//   // logMsg.source.data = (uint8_t*)test;
//   // logMsg.source.len = 0;
//   // logMsg.text.data = (uint8_t*)msg;
//   // logMsg.text.len = 0;

//   uint8_t send_buf[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
//   uint32_t send_len = UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE;//uavcan_protocol_debug_LogMessage_encode(&logMsg, send_buf);

//   uint8_t debug_transfer_id = 100;
//   //canardBroadcast(&iface->canard, UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE, UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID, &debug_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, send_buf, send_len);
//   //chEvtBroadcast(&iface->tx_request);
//   /*uavcanBroadcast(iface,
//                                               UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
//                                               UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
//                                               &debug_transfer_id,
//                                               CANARD_TRANSFER_PRIORITY_LOW,
//                                               send_buf,
//                                               send_len);*/
// }

/*
 * Receiver thread.
 */
static THD_FUNCTION(can_rx, p) {
  event_listener_t el;
  CANRxFrame rx_msg;
  CanardCANFrame rx_frame;
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)p;

  chRegSetThreadName("can_rx");
  chEvtRegister(&iface->can_driver->rxfull_event, &el, 0);
  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0)
      continue;

    // Wait untial a CAN message is received
    while (canReceive(iface->can_driver, CAN_ANY_MAILBOX, &rx_msg, TIME_IMMEDIATE) == MSG_OK) {
      // Process message.
      const uint32_t timestamp = TIME_I2US(chVTGetSystemTimeX());
      memcpy(rx_frame.data, rx_msg.data8, 8);
      rx_frame.data_len = rx_msg.DLC;
      if(rx_msg.IDE) {
        rx_frame.id = CANARD_CAN_FRAME_EFF | rx_msg.EID;
      } else {
        rx_frame.id = rx_msg.SID;
      }
 
      // Let canard handle the frame
      chMtxLock(&iface->mutex);
      canardHandleRxFrame(&iface->canard, &rx_frame, timestamp);
      chMtxUnlock(&iface->mutex);
    }
  }
  chEvtUnregister(&iface->can_driver->rxfull_event, &el);
}

/*
 * Transmitter thread.
 */
static THD_FUNCTION(can_tx, p) {
  event_listener_t txc, txe, txr;
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)p;
  uint8_t err_cnt = 0;

  chRegSetThreadName("can_tx");
  chEvtRegister(&iface->can_driver->txempty_event, &txc, 0);
  chEvtRegister(&iface->can_driver->error_event, &txe, 1);
  chEvtRegister(&iface->tx_request, &txr, 2);

  while (true) {
    eventmask_t evts = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    // Successfull transmit
    if (evts == 0)
      continue;

    // Transmit error
    if(evts & EVENT_MASK(1))
    {
      iface->transmit_err_cnt++;
      chEvtGetAndClearFlags(&txe);
      continue;
    }

    chMtxLock(&iface->mutex);
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&iface->canard)) != NULL;) {
      CANTxFrame tx_msg;
      tx_msg.DLC = txf->data_len;
      memcpy(tx_msg.data8, txf->data, 8);
      tx_msg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
      tx_msg.IDE = CAN_IDE_EXT;
      tx_msg.RTR = CAN_RTR_DATA;
      if (canTransmit(iface->can_driver, CAN_ANY_MAILBOX, &tx_msg, TIME_IMMEDIATE) == MSG_OK) {
        err_cnt = 0;
        canardPopTxQueue(&iface->canard);
      } else {
        // After 5 retries giveup and clean full queue
        if(err_cnt >= 5) {
          err_cnt = 0;
          iface->transmit_err_flush_cnt++;
          while (canardPeekTxQueue(&iface->canard)) { canardPopTxQueue(&iface->canard); }
          continue;
        }

        // Timeout - just exit and try again later
        chMtxUnlock(&iface->mutex);
        chThdSleepMilliseconds(++err_cnt);
        chMtxLock(&iface->mutex);
        continue;
      }
    }
    chMtxUnlock(&iface->mutex);
  }
}

/*
 * UAVCan thread.
 */
static THD_FUNCTION(uavcan_thrd, p) {
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)p;

  chRegSetThreadName("uavcan");

  // Try to get a Node ID
  uint8_t node_id_allocation_transfer_id = 0;
  while(canardGetLocalNodeID(&iface->canard) == CANARD_BROADCAST_NODE_ID) {
    iface->send_next_node_id_allocation_request_at_ms =
      TIME_I2MS(chVTGetSystemTimeX()) + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
      (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    while ((TIME_I2MS(chVTGetSystemTimeX()) < iface->send_next_node_id_allocation_request_at_ms) &&
          (canardGetLocalNodeID(&iface->canard) == CANARD_BROADCAST_NODE_ID))
    {
      chMtxLock(&iface->mutex);
      canardCleanupStaleTransfers(&iface->canard, TIME_I2MS(chVTGetSystemTimeX()));
      chMtxUnlock(&iface->mutex);
      chThdSleepMilliseconds(50);
    }

    if (canardGetLocalNodeID(&iface->canard) != CANARD_BROADCAST_NODE_ID)
    {
      break;
    }

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(iface->node_id << 1U);

    if (iface->node_id_allocation_unique_id_offset == 0)
    {
      allocation_request[0] |= 1;     // First part of unique ID
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[16] = {0};
    memcpy(my_unique_id, (void *)UID_BASE, 12);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - iface->node_id_allocation_unique_id_offset);
    if (uid_size > MaxLenOfUniqueIDInRequest)
    {
      uid_size = MaxLenOfUniqueIDInRequest;
    }

    // Paranoia time
    memmove(&allocation_request[1], &my_unique_id[iface->node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = uavcanBroadcast(iface,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t) (uid_size + 1));
    if (bcast_res < 0)
    {
      //printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    iface->node_id_allocation_unique_id_offset = 0;
  }

  while(true) {
    broadcast_node_status(iface);

    chMtxLock(&iface->mutex);
    canardCleanupStaleTransfers(&iface->canard, TIME_I2MS(chVTGetSystemTimeX()));
    chMtxUnlock(&iface->mutex);

    chThdSleepMilliseconds(500);
  }
}

static void handle_allocation_response(struct uavcan_iface_t *iface, CanardRxTransfer* transfer)
{
  // Rule C - updating the randomized time interval
  iface->send_next_node_id_allocation_request_at_ms =
    TIME_I2MS(chVTGetSystemTimeX()) + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
    (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

  if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
  {
    iface->node_id_allocation_unique_id_offset = 0;
    return;
  }

  // Copying the unique ID from the message
  struct uavcan_protocol_dynamic_node_id_Allocation msg;
  if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
    /* bad packet */
    return;
  }

  // Obtaining the local unique ID
  uint8_t my_unique_id[16] = {0};
  memcpy(my_unique_id, (void *)UID_BASE, 12);

  // Matching the received UID against the local one
  if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
    //printf("Mismatching allocation response\n");
    iface->node_id_allocation_unique_id_offset = 0;
    return;         // No match, return
  }

  if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
    // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
    iface->node_id_allocation_unique_id_offset = msg.unique_id.len;
    iface->send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

    //printf("Matching allocation response: %d\n", received_unique_id_len);
  } else {
    // Allocation complete - copying the allocated node ID from the message
    canardSetLocalNodeID(&iface->canard, msg.node_id);
    //printf("Node ID allocated: %d\n", allocated_node_id);
  }
}

/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
  (void)source_node_id;

  if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
  {
    /*
     * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
     */
    if ((transfer_type == CanardTransferTypeBroadcast) &&
      (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
    {
      *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
      return true;
    }
    return false;
  }


  switch (data_type_id) {
    case UAVCAN_PROTOCOL_NODESTATUS_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
      return true;
    case UAVCAN_EQUIPMENT_ESC_STATUS_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE;
      return true;
    case UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE;
      return true;
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
      return true;
    case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
      return true;
    case UAVCAN_TUNNEL_CALL_ID:
      *out_data_type_signature = UAVCAN_TUNNEL_CALL_SIGNATURE;
      return true;

    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
      return true;
  }
  

  return false;
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)ins->user_reference;

  /*
   * Dynamic node ID allocation protocol.
   * Taking this branch only if we don't have a node ID, ignoring otherwise.
   */
  if ((canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) &&
    (transfer->transfer_type == CanardTransferTypeBroadcast) &&
    (transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
  {
    handle_allocation_response(iface, transfer);
    return;
  }

  // First handle the packages internally
  switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
      handle_esc_rawcommand(iface, transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      handle_param_getset(iface, transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      handle_param_execute_opcode(iface, transfer);
      break;
    case UAVCAN_TUNNEL_CALL_ID:
      //handle_tunnel_call(iface, transfer);
      break;

    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      handle_get_node_info(iface, transfer);
      break;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
      servos_disable();
      *((uint32_t *)0x20004FF0) = 0xDEADBEEF;
      NVIC_SystemReset();
      break;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      servos_disable();
      NVIC_SystemReset();
      break;
  }

  // Forward if needed
#if defined(STM32_CAN_USE_CAN1) && defined(STM32_CAN_USE_CAN2)
  struct uavcan_iface_t *main_iface = NULL;
  struct uavcan_iface_t *other_iface = NULL;

  // Set the correct bridge
  if(uavcan_bridge == UAVCAN_BRIDGE_CAN1TOCAN2) {
    main_iface = &can1_iface;
    other_iface = &can2_iface;
  } else if(uavcan_bridge == UAVCAN_BRIDGE_CAN2TOCAN1) {
    main_iface = &can2_iface;
    other_iface = &can1_iface;
  } else {
    return;
  }

  // Get the signature and verify broadcast
  uint64_t data_type_signature;
  if(!shouldAcceptTransfer(&can1_iface.canard, &data_type_signature, transfer->data_type_id, transfer->transfer_type, transfer->source_node_id))
    return;
  if(transfer->transfer_type != CanardTransferTypeBroadcast)
    return;

  // Weird payload handling
  uint8_t tx_payload[transfer->payload_len];
  memcpy(tx_payload, transfer->payload_head, CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE);
  uint8_t offset = CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE;
  uint16_t remaining = transfer->payload_len - CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE;
  CanardBufferBlock* block = transfer->payload_middle;
  while(block) {
    uint8_t bsize = (remaining < CANARD_BUFFER_BLOCK_DATA_SIZE) ? remaining : CANARD_BUFFER_BLOCK_DATA_SIZE;
    memcpy(&tx_payload[offset], block->data, bsize);
    offset += CANARD_BUFFER_BLOCK_DATA_SIZE;
    remaining -= CANARD_BUFFER_BLOCK_DATA_SIZE;
    block = block->next;
  }
  if(transfer->payload_tail && remaining > 0) {
    memcpy(&tx_payload[offset], transfer->payload_tail, remaining);
  }

  // Trafic from Main -> Sub    (UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID)
  if(iface == main_iface && 
      (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID
      || transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID)) {
    uavcanBroadcast(other_iface,
                    data_type_signature,
                    transfer->data_type_id,
                    &transfer->transfer_id,
                    transfer->priority,
                    tx_payload,
                    transfer->payload_len);
  } 
  // Trafic from Sub -> Main     (UAVCAN_PROTOCOL_NODESTATUS_ID, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID)
  else if(iface == other_iface && 
      (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_STATUS_ID
      || transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID)) {
    uavcanBroadcast(main_iface,
                    data_type_signature,
                    transfer->data_type_id,
                    &transfer->transfer_id,
                    transfer->priority,
                    tx_payload,
                    transfer->payload_len);
  }
#endif
}

/**
 * Try to compute the timing registers for the can interface and set the configuration
 */
static bool uavcanConfigureIface(struct uavcan_iface_t *iface)
{
  if (iface->can_baudrate < 1) {
    return false;
  }

  // Hardware configurationn
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  const uint32_t pclk = STM32_FDCANCLK;
#else
  const uint32_t pclk = STM32_PCLK1;
#endif
  static const int MaxBS1 = 16;
  static const int MaxBS2 = 8;

  /*
    * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    *      CAN in Automation, 2003
    *
    * According to the source, optimal quanta per bit are:
    *   Bitrate        Optimal Maximum
    *   1000 kbps      8       10
    *   500  kbps      16      17
    *   250  kbps      16      17
    *   125  kbps      16      17
    */
  const int max_quanta_per_bit = (iface->can_baudrate >= 1000000) ? 10 : 17;
  static const int MaxSamplePointLocation = 900;

  /*
    * Computing (prescaler * BS):
    *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    * let:
    *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    *   PRESCALER_BS = PRESCALER * BS
    * ==>
    *   PRESCALER_BS = PCLK / BITRATE
    */
  const uint32_t prescaler_bs = pclk / iface->can_baudrate;

// Searching for such prescaler value so that the number of quanta per bit is highest.
  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;
  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
    if (bs1_bs2_sum <= 2) {
      return false;          // No solution
    }
    bs1_bs2_sum--;
  }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
  if ((prescaler < 1U) || (prescaler > 1024U)) {
    return false;              // No solution
  }

  /*
    * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    * We need to find the values so that the sample point is as close as possible to the optimal value.
    *
    *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    *   {{bs2 -> (1 + bs1)/7}}
    *
    * Hence:
    *   bs2 = (1 + bs1) / 7
    *   bs1 = (7 * bs1_bs2_sum - 1) / 8
    *
    * Sample point location can be computed as follows:
    *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    *
    * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    *   - With rounding to nearest
    *   - With rounding to zero
    */
// First attempt with rounding to nearest
  uint8_t bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = bs1_bs2_sum - bs1;
  uint16_t sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);

// Second attempt with rounding to zero
  if (sample_point_permill > MaxSamplePointLocation) {
    bs1 = (7 * bs1_bs2_sum - 1) / 8;
    bs2 = bs1_bs2_sum - bs1;
    sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);
  }

  /*
    * Final validation
    * Helpful Python:
    * def sample_point_from_btr(x):
    *     assert 0b0011110010000000111111000000000 & x == 0
    *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
    *     return (1+ts1+1)/(1+ts1+1+ts2+1)
    *
    */
  if ((iface->can_baudrate != (pclk / (prescaler * (1 + bs1 + bs2)))) || (bs1 < 1) || (bs1 > MaxBS1) || (bs2 < 1)
      || (bs2 > MaxBS2)) {
    return false;
  }

  // Configure the interface
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  iface->can_cfg.NBTP = (0 << FDCAN_NBTP_NSJW_Pos) | ((bs1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((
                          bs2 - 1) << FDCAN_NBTP_NTSEG2_Pos) | ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos);
  iface->can_cfg.CCCR = FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;
#else
  iface->can_cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  iface->can_cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(bs1 - 1) | CAN_BTR_TS2(bs2 - 1) | CAN_BTR_BRP(prescaler - 1);
#endif
  return true;
}

static void uavcanInitIface(struct uavcan_iface_t *iface) {
  // First try to configure abort if failed
  if (!uavcanConfigureIface(iface)) {
    return;
  }

  // Initialize mutexes/events for multithread locking
  chMtxObjectInit(&iface->mutex);
  chEvtObjectInit(&iface->tx_request);

  // Initialize canard
  canardInit(&iface->canard, iface->canard_memory_pool, sizeof(iface->canard_memory_pool),
    onTransferReceived, shouldAcceptTransfer, iface);

  // Set the node ID from the config
  iface->node_id = config_get_by_name("NODE id", 0)->val.i;
  if(iface->node_id != CANARD_BROADCAST_NODE_ID)
    canardSetLocalNodeID(&iface->canard, iface->node_id);

  // Start the can interface
  canStart(iface->can_driver, &iface->can_cfg);

  // Start the receiver and transmitter thread
  chThdCreateStatic(iface->thread_rx_wa, iface->thread_rx_wa_size, NORMALPRIO + 8, can_rx, (void*)iface);
  chThdCreateStatic(iface->thread_tx_wa, iface->thread_tx_wa_size, NORMALPRIO + 7, can_tx, (void*)iface);
  chThdCreateStatic(iface->thread_uavcan_wa, iface->thread_uavcan_wa_size, NORMALPRIO + 6, uavcan_thrd, (void*)iface);
}

/**
 * Initialization of the CAN driver
 */
void uavcanInit(void) {
  // Get the configuration variables
  uint8_t can_termination = config_get_by_name("CAN termination", 0)->val.i;
  uavcan_bridge = config_get_by_name("CAN bridge", 0)->val.i;

#if defined(CAN1_TERM_LINE) // High is closed
  if(can_termination & 0x1)
    palSetLine(CAN1_TERM_LINE);
  else
    palClearLine(CAN1_TERM_LINE);
#endif

#if defined(CAN2_TERM_LINE) // High is open
  if(can_termination & 0x2)
    palClearLine(CAN2_TERM_LINE);
  else
    palSetLine(CAN2_TERM_LINE);
#endif

  // Activate the interfaces
#if STM32_CAN_USE_CAN1
#if defined(CAN1_STBY_LINE)
  palClearLine(CAN1_STBY_LINE);
#endif
  uavcanInitIface(&can1_iface);
#endif
#if STM32_CAN_USE_CAN2
#if defined(CAN2_STBY_LINE)
  palClearLine(CAN2_STBY_LINE);
#endif
  uavcanInitIface(&can2_iface);
#endif
}
