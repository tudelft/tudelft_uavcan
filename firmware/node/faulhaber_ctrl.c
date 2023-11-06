#include "faulhaber_ctrl.h"

#include "config.h"
#include <stdlib.h>
#include <math.h>

enum faulhaber_parser_states_t {
    UINIT,
    GOT_SOF,
    GOT_LENGTH,
    GOT_NODE_NB,
    GET_DATA,
    GET_CRC,
    GET_EOF,
    GOT_FULL_PACKET
};

struct faulhaber_parser_t {
  enum faulhaber_parser_states_t state;
  uint8_t node_nb;
  uint8_t cmd_code;
  uint8_t data_length;
  uint8_t data_idx;
  uint8_t data[64];

  uint8_t calc_crc8;
};
static struct faulhaber_parser_t faulhaber_p;
struct faulhaber_ctrl_t faulhaber_ctrl;
static void faulhaber_parser(struct faulhaber_parser_t *p, uint8_t c);
static void faulhaber_send_command(uint8_t cmd_code, uint8_t *data, uint8_t data_length) ;

static THD_WORKING_AREA(faulhaber_ctrl_rx_wa, 512);
static THD_FUNCTION(faulhaber_ctrl_rx_thd, arg) {
  (void)arg;
  chRegSetThreadName("faulhaber_ctrl_rx");
  faulhaber_p.state = UINIT;

  while (true) {
    size_t recv_size = 128;
    uint8_t buf[128];

    // Try to receive bytes
    if(uartReceiveTimeout(faulhaber_ctrl.port, &recv_size, (void *)buf, TIME_MS2I(100)) != MSG_RESET) {
        for(uint8_t i = 0; i < recv_size; i++) {
            faulhaber_parser(&faulhaber_p, buf[i]);

            // When we received a full message parse it
            if(faulhaber_p.state == GOT_FULL_PACKET) {
              // Parse the actual position message
              if(faulhaber_p.cmd_code == 0x01 && faulhaber_p.data[0] == 0x64 && faulhaber_p.data[1] == 0x60 && faulhaber_p.data[2] == 0x00) {
                faulhaber_ctrl.actual_position = faulhaber_p.data[3] | (faulhaber_p.data[4] << 8) | (faulhaber_p.data[5] << 16) | (faulhaber_p.data[6] << 24);
              }

              // Parse the statuscode message
              if(faulhaber_p.cmd_code == 0x05) {
                uint16_t status_code = faulhaber_p.data[0] | (faulhaber_p.data[1] << 8);

                // Homing
                if(!faulhaber_ctrl.homing_completed && status_code&0x1000 && status_code&0x400)
                  faulhaber_ctrl.homing_completed = true;
                
                // Position accepted
                if(!faulhaber_ctrl.position_ready && status_code&0x1000)
                  faulhaber_ctrl.position_ready = true;
                  
                // Target reached
                if(!faulhaber_ctrl.target_reached && status_code&0x400)
                  faulhaber_ctrl.target_reached = true;
              }

              // Start receiving new messages
              faulhaber_p.state = UINIT;
            }
        }
    }
  }
}

static THD_WORKING_AREA(faulhaber_ctrl_tx_wa, 1024);
static THD_FUNCTION(faulhaber_ctrl_tx_thd, arg) {
  (void)arg;
  chRegSetThreadName("faulhaber_ctrl_tx");

  // First we sleep until the start timeout is done
  chThdSleepSeconds(faulhaber_ctrl.start_timeout_s);

  // Try to home
  faulhaber_ctrl.homing_completed = false;
  static uint8_t data0[] = { 0x40, 0x60, 0x00, 0x0E, 0x00}; // Set 0x6040.00 to 0x000E: Try to start
  faulhaber_send_command(0x02, data0, 5);
  chThdSleepMilliseconds(10);
  static uint8_t data1[] = { 0x60, 0x60, 0x00, 0x06 }; // Set 0x6060.00 to 0x06:  Homing mode
  faulhaber_send_command(0x02, data1, 4);
  chThdSleepMilliseconds(10);
  uint8_t data2[] = { 0x98, 0x60, 0x00, faulhaber_ctrl.home_method }; // Set 0x6098.00 to 0x11:  Homing method to ??
  faulhaber_send_command(0x02, data2, 4);
  chThdSleepMilliseconds(10);
  static uint8_t data3[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
  faulhaber_send_command(0x02, data3, 5);
  chThdSleepMilliseconds(10);
  static uint8_t data4[] = { 0x40, 0x60, 0x00, 0x1F, 0x00}; // Set 0x6040.00 to 0x001F: Start moving
  faulhaber_send_command(0x02, data4, 5);

  // Wait till homing is finished
  systime_t start = chVTGetSystemTimeX();
  while(!faulhaber_ctrl.homing_completed) {
    chThdSleepMilliseconds(50);

    // Timeout after 25 seconds
    if(chVTTimeElapsedSinceX(start) > TIME_S2I(25)) {
      // TODO
      break;
    }
  }

  // Disable the drives
  /*static uint8_t data5[] = { 0x40, 0x60, 0x00, 0x0D, 0x00}; // Set 0x6040.00 to 0x000D: Disable drives
  faulhaber_send_command(0x02, data5, 5);*/

  // Change to position mode
  static uint8_t data7[] = { 0x60, 0x60, 0x00, 0x01 }; // Set 0x6060.00 to 0x01:  Position mode
  faulhaber_send_command(0x02, data7, 4);
  chThdSleepMilliseconds(10);
  static uint8_t data10[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
  faulhaber_send_command(0x02, data10, 5);
  chThdSleepMilliseconds(10);
  faulhaber_ctrl.position_ready = true;

  // Set target position to not set
  int32_t current_target_position = INT32_MAX;
  faulhaber_ctrl.target_position = INT32_MAX;

  while (true) {
    // Request the current position
    static uint8_t data6[] = {0x64, 0x60, 0x00}; // Get 0x6064.00: Get the actual position
    faulhaber_send_command(0x01, data6, 3);
    chThdSleepMilliseconds(10);

    // Change position if needed
    if(faulhaber_ctrl.target_position != INT32_MAX && faulhaber_ctrl.target_position != current_target_position) {
      faulhaber_ctrl.position_ready = false;
      faulhaber_ctrl.target_reached = false;
      uint8_t data8[] = {0x7A, 0x60, 0x00, (faulhaber_ctrl.target_position & 0xFF), ((faulhaber_ctrl.target_position >> 8) & 0xFF), ((faulhaber_ctrl.target_position >> 16) & 0xFF), ((faulhaber_ctrl.target_position >> 24) & 0xFF) }; // Set 0x607A.00 to 0x00000000:  Target position 0
      faulhaber_send_command(0x02, data8, 7);
      chThdSleepMilliseconds(10);
      static uint8_t data11[] = {0x40, 0x60, 0x00, 0x3F, 0x00}; // Set 0x6040.00 to 0x003F: Start moving (immediate)
      faulhaber_send_command(0x02, data11, 5);

      // Wait for response for accepting new positions
      start = chVTGetSystemTimeX();
      while(!faulhaber_ctrl.position_ready) {
        chThdSleepMilliseconds(2);

        // Timeout after 200 milliseconds
        if(chVTTimeElapsedSinceX(start) > TIME_MS2I(200)) {
          // TODO
          break;
        }
      }

      // Reset for next operation
      current_target_position = faulhaber_ctrl.target_position;
      static uint8_t data12[] = {0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
      faulhaber_send_command(0x02, data12, 5);
      chThdSleepMilliseconds(10);
    }
   
    chThdSleepMilliseconds(90);
  }
}

static void faulhaber_ctrl_broadcast_status(void) {
  // Set the values
  int64_t range = (faulhaber_ctrl.max_pos - faulhaber_ctrl.min_pos);
  uavcan_equipment_actuator_Status actuatorStatus;
  actuatorStatus.actuator_id = faulhaber_ctrl.index;
  actuatorStatus.position = (double)faulhaber_ctrl.actual_position / (double)range * M_PI_2;

  uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
  uint16_t total_size = uavcan_equipment_actuator_Status_encode(&actuatorStatus, buffer);

  static uint8_t transfer_id;
  uavcanBroadcastAll(
      UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
      UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID, &transfer_id,
      CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
}

static THD_WORKING_AREA(faulhaber_ctrl_telem_send_wa, 512);
static THD_FUNCTION(faulhaber_ctrl_telem_send_thd, arg) {
  (void)arg;
  chRegSetThreadName("faulhaber_ctrl_telem");
  while (true) {
    faulhaber_ctrl_broadcast_status();
    chThdSleepMilliseconds(faulhaber_ctrl.telem_vt_delay);
  }
}

void faulhaber_ctrl_init(void) {
    // Get the configuration
    faulhaber_ctrl.index = config_get_by_name("FAULHABER index", 0)->val.i;
    faulhaber_ctrl.telem_vt_delay = 1000.f / config_get_by_name("FAULHABER telem frequency", 0)->val.f;
    faulhaber_ctrl.node_nb = config_get_by_name("FAULHABER node number", 0)->val.i;
    faulhaber_ctrl.start_timeout_s = config_get_by_name("FAULHABER start timeout (s)", 0)->val.i;
    faulhaber_ctrl.home_method = config_get_by_name("FAULHABER home method", 0)->val.i;
    faulhaber_ctrl.deadband = config_get_by_name("FAULHABER deadband", 0)->val.i;
    faulhaber_ctrl.min_pos = config_get_by_name("FAULHABER min position", 0)->val.i;
    faulhaber_ctrl.max_pos = config_get_by_name("FAULHABER max position", 0)->val.i;
    uint32_t baudrate = config_get_by_name("FAULHABER baudrate", 0)->val.i;
    

    // The UART port settings
    UARTConfig uart_cfg = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        baudrate,
        USART_CR1_UE | USART_CR1_RE | USART_CR1_TE,
        0,
        0
    };

    // Configure the port
    uint8_t port = config_get_by_name("FAULHABER port", 0)->val.i;
    if(port == 1) {
        palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL1_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        faulhaber_ctrl.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        faulhaber_ctrl.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL3_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        faulhaber_ctrl.port = &UARTD3;
    } else{
        faulhaber_ctrl.port = NULL;
    }

    // Open the telemetry port and start the thread
    if(faulhaber_ctrl.port != NULL) {
        uartStart(faulhaber_ctrl.port, &uart_cfg);
        chThdCreateStatic(faulhaber_ctrl_rx_wa, sizeof(faulhaber_ctrl_rx_wa), NORMALPRIO+1, faulhaber_ctrl_rx_thd, NULL);
        chThdCreateStatic(faulhaber_ctrl_tx_wa, sizeof(faulhaber_ctrl_tx_wa), NORMALPRIO+2, faulhaber_ctrl_tx_thd, NULL);
        chThdCreateStatic(faulhaber_ctrl_telem_send_wa, sizeof(faulhaber_ctrl_telem_send_wa), NORMALPRIO-6, faulhaber_ctrl_telem_send_thd, NULL);
    }
}


static uint8_t faulhaber_crc8_byte(uint8_t u8Byte, uint8_t u8CRC)
{
  uint8_t i;
  u8CRC = u8CRC ^ u8Byte;
  for (i = 0; i < 8; i++) {
    if (u8CRC & 0x01) {
    u8CRC = (u8CRC >> 1) ^ 0xD5;
    }
    else {
    u8CRC >>= 1;
    }
  }
  return u8CRC;
}

static uint8_t faulhaber_crc8(uint8_t *data, uint8_t data_length) {
    uint8_t crc8 = 0xFF; // Start CRC with 0xFF
    for(uint8_t i = 0; i < data_length; i++) {
        crc8 = faulhaber_crc8_byte(data[i], crc8);
    }
    return crc8;
}

static void faulhaber_send_command(uint8_t cmd_code, uint8_t *data, uint8_t data_length) {
    uint8_t msg[128];
    msg[0] = 'S'; // Character (S) as Start of Frame
    msg[1] = data_length + 4; // Telegram length without SOF/EOF (packet length)
    msg[2] = faulhaber_ctrl.node_nb; // Node number of the slave (0 = Broadcast)
    msg[3] = cmd_code; // See Tab. 2
    
    for(uint8_t i = 0; i < data_length; i++) { // Data area (length = packet length – 4)
        msg[4 + i] = data[i];
    }

    msg[4 + data_length] = faulhaber_crc8(&msg[1], data_length + 3); // CRC8 with polynomial 0xD5 over byte 2–N
    msg[5 + data_length] = 'E'; // Character (E) as End of Frame

    size_t send_size = 6 + data_length;
    uartSendFullTimeout(faulhaber_ctrl.port, &send_size, (void *)msg, TIME_MS2I(350));
}

static void faulhaber_parser(struct faulhaber_parser_t *p, uint8_t c) {
  switch (p->state) {
    case UINIT:
      if (c == 'S') {
        p->state = GOT_SOF;
        p->calc_crc8 = 0xFF;
      }
      break;
    case GOT_SOF:
      if(c - 4 < 0) {
        p->state = UINIT;
      } else {
        p->data_length = c - 4;
        p->calc_crc8 = faulhaber_crc8_byte(c, p->calc_crc8);
        p->state = GOT_LENGTH;
      }
      break;
    case GOT_LENGTH:
      p->node_nb = c;
      p->calc_crc8 = faulhaber_crc8_byte(c, p->calc_crc8);
      p->state = GOT_NODE_NB;
      break;
    case GOT_NODE_NB:
      p->cmd_code = c;
      p->data_idx = 0;
      p->calc_crc8 = faulhaber_crc8_byte(c, p->calc_crc8);
      p->state = GET_DATA;
      break;
    case GET_DATA:
      p->data[p->data_idx++] = c;
      p->calc_crc8 = faulhaber_crc8_byte(c, p->calc_crc8);
      if(p->data_idx >= p->data_length)
        p->state = GET_CRC;
      break;
    case GET_CRC:
      if(p->calc_crc8 == c) {
        p->state = GET_EOF;
      } else {
        p->state = UINIT;
      }
      break;
    case GET_EOF:
      if(c == 'E') {
        p->state = GOT_FULL_PACKET;
      } else {
        p->state = UINIT;
      }
      break;
    default:
      p->state = UINIT;
      break;
  }
}