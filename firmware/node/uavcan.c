#include <string.h>
#include <stdlib.h>

#include "uavcan.h"
#include "config.h"
#include "node.h"
#include "esc.h"

#if STM32_CAN_USE_CAN1
static THD_WORKING_AREA(can1_rx_wa, 1024*2);
static THD_WORKING_AREA(can1_tx_wa, 1024*2);
static THD_WORKING_AREA(can1_uavcan_wa, 2048*2);

static struct uavcan_iface_t can1_iface = {
  .can_driver = &CAND1,
  .can_cfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(14) | CAN_BTR_BRP((STM32_PCLK1/18)/125000 - 1)
  },
  .thread_rx_wa = can1_rx_wa,
  .thread_rx_wa_size = sizeof(can1_rx_wa),
  .thread_tx_wa = can1_tx_wa,
  .thread_tx_wa_size = sizeof(can1_tx_wa),
  .thread_uavcan_wa = can1_uavcan_wa,
  .thread_uavcan_wa_size = sizeof(can1_uavcan_wa),
  .node_id = CANARD_BROADCAST_NODE_ID
};
#endif

#if STM32_CAN_USE_CAN2
static THD_WORKING_AREA(can2_rx_wa, 1024*2);
static THD_WORKING_AREA(can2_tx_wa, 1024*2);
static THD_WORKING_AREA(can2_uavcan_wa, 2048*2);

static struct uavcan_iface_t can2_iface = {
  .can_driver = &CAND2,
  .can_cfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(14) | CAN_BTR_BRP((STM32_PCLK1/18)/125000 - 1)
  },
  .thread_rx_wa = can2_rx_wa,
  .thread_rx_wa_size = sizeof(can2_rx_wa),
  .thread_tx_wa = can2_tx_wa,
  .thread_tx_wa_size = sizeof(can2_tx_wa),
  .thread_uavcan_wa = can2_uavcan_wa,
  .thread_uavcan_wa_size = sizeof(can2_uavcan_wa),
  .node_id = CANARD_BROADCAST_NODE_ID
};
#endif

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
  chEvtRegister(&iface->can_driver->rxfull_event, &el, EVENT_MASK(0));
  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0)
      continue;
    chMtxLock(&iface->mutex);
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
 
      canardHandleRxFrame(&iface->canard, &rx_frame, timestamp);
    }
    chMtxUnlock(&iface->mutex);
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
  chEvtRegister(&iface->can_driver->txempty_event, &txc, EVENT_MASK(0));
  chEvtRegister(&iface->can_driver->error_event, &txe, EVENT_MASK(1));
  chEvtRegister(&iface->tx_request, &txr, EVENT_MASK(2));

  while (true) {
    eventmask_t evts = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    if (evts == 0)
      continue;

    if(evts & EVENT_MASK(1))
    {
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
        // After 100 retries giveup
        if(err_cnt >= 100) {
          err_cnt = 0;
          canardPopTxQueue(&iface->canard);
          continue;
        }

        // Timeout - just exit and try again later
        err_cnt++;
        chThdSleepMilliseconds(err_cnt * 5);
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
    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH] = {0};
    memcpy(my_unique_id, (void *)UID_BASE, 12);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH - iface->node_id_allocation_unique_id_offset);
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

    //uartStartSend(&UARTD3, 13, "Starting...\r\n");
    //palToggleLine(RS485_DE_LINE);
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
  static const uint8_t UniqueIDBitOffset = 8;
  uint8_t received_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
  uint8_t received_unique_id_len = 0;
  for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++) {
    //assert(received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
    const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
    (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
  }

  // Obtaining the local unique ID
  uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH] = {0};
  memcpy(my_unique_id, (void *)UID_BASE, 12);

  // Matching the received UID against the local one
  if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0) {
    //printf("Mismatching allocation response\n");
    iface->node_id_allocation_unique_id_offset = 0;
    return;         // No match, return
  }

  if (received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH) {
    // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
    iface->node_id_allocation_unique_id_offset = received_unique_id_len;
    iface->send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

    //printf("Matching allocation response: %d\n", received_unique_id_len);
  } else {
    // Allocation complete - copying the allocated node ID from the message
    uint8_t allocated_node_id = 0;
    (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
    //assert(allocated_node_id <= 127);

    canardSetLocalNodeID(&iface->canard, allocated_node_id);
    //printf("Node ID allocated: %d\n", allocated_node_id);
  }
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

  switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
      handle_esc_rawcommand(iface, transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      handle_param_getset(iface, transfer);
      break;

    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      handle_get_node_info(iface, transfer);
      break;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
      *((uint32_t *)0x20004FF0) = 0xDEADBEEF;
      NVIC_SystemReset();
      break;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      NVIC_SystemReset();
      break;
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
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
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

static void uavcanInitIface(struct uavcan_iface_t *iface) {
  chMtxObjectInit(&iface->mutex);
  chEvtObjectInit(&iface->tx_request);
  canardInit(&iface->canard, iface->canard_memory_pool, sizeof(iface->canard_memory_pool),
    onTransferReceived, shouldAcceptTransfer, iface);

  // Set the node ID from the config
  iface->node_id = config_get_by_name("NODE id", 0)->val.i;
  if(iface->node_id != CANARD_BROADCAST_NODE_ID)
    canardSetLocalNodeID(&iface->canard, iface->node_id);

  canStart(iface->can_driver, &iface->can_cfg);
  chThdCreateStatic(iface->thread_rx_wa, iface->thread_rx_wa_size, NORMALPRIO + 8, can_rx, (void*)iface);
  chThdCreateStatic(iface->thread_tx_wa, iface->thread_tx_wa_size, NORMALPRIO + 7, can_tx, (void*)iface);
  chThdCreateStatic(iface->thread_uavcan_wa, iface->thread_uavcan_wa_size, NORMALPRIO + 6, uavcan_thrd, (void*)iface);
}

/**
 * Initialization of the CAN driver
 */
void uavcanInit(void) {
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