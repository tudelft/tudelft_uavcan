#include "esc_telem.h"

#include "config.h"


struct esc_telem_t esc_telem = {0};
static void esc_telem_parse_tmotorf(uint8_t msg[], uint8_t len);
static void esc_telem_parse_tmotor_flame(uint8_t msg[], uint8_t len);
static void esc_telem_parse_tmotor_alpha(uint8_t msg[], uint8_t len);

static THD_WORKING_AREA(esc_telem_wa, 512);
static THD_FUNCTION(esc_telem_thd, arg) {
  (void)arg;
  chRegSetThreadName("esc_telem");
  while (true) {
    size_t recv_size = (esc_telem.type == 0)? 10:24;
    uint8_t buf[32];

    // Try to receive bytes
    if(uartReceiveTimeout(esc_telem.port, &recv_size, (void *)buf, TIME_MS2I(40)) == MSG_OK) {
        if(esc_telem.type == 0)
            esc_telem_parse_tmotorf(buf, recv_size);
        else if(esc_telem.type == 1)
            esc_telem_parse_tmotor_flame(buf, recv_size);
        else if(esc_telem.type == 2)
            esc_telem_parse_tmotor_alpha(buf, recv_size);
    }
  }
}

static void esc_telem_broadcast_status(void) {
  // Inhibit when no telemetry is received
  if(!esc_telem.data.received) {
    esc_telem.data.timeout_cnt++;
    return;
  }

  // Set the values
  struct uavcan_equipment_esc_Status escStatus;
  escStatus.error_count = esc_telem.data.timeout_cnt;
  escStatus.voltage = esc_telem.data.voltage;
  escStatus.current = esc_telem.data.current;
  escStatus.temperature = esc_telem.data.temp + 274.15f;
  escStatus.rpm = esc_telem.data.erpm / esc_telem.pole_pairs;
  escStatus.esc_index = esc_telem.index;

  uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
  uint16_t total_size = uavcan_equipment_esc_Status_encode(&escStatus, buffer);

  static uint8_t transfer_id;
  uavcanBroadcastAll(
      UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
      UAVCAN_EQUIPMENT_ESC_STATUS_ID, &transfer_id,
      CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
  esc_telem.data.received = false;
}

static THD_WORKING_AREA(esc_telem_send_wa, 512);
static THD_FUNCTION(esc_telem_send_thd, arg) {
  (void)arg;
  chRegSetThreadName("esc_telem");
  while (true) {
    esc_telem_broadcast_status();
    chThdSleepMilliseconds(esc_telem.vt_delay);
  }
}


void esc_telem_init(void) {
    // Get the configuration
    esc_telem.index = config_get_by_name("ESC telem index", 0)->val.i;
    esc_telem.vt_delay = 1000.f / config_get_by_name("ESC telem frequency", 0)->val.f;
    esc_telem.type = config_get_by_name("ESC telem type", 0)->val.i;
    esc_telem.pole_pairs = config_get_by_name("ESC telem pole pairs", 0)->val.i;
    uint8_t port = config_get_by_name("ESC telem port", 0)->val.i;

    // Change port settings based on type
    UARTConfig uart_cfg = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        (esc_telem.type == 0)? 115200 : 19200,
        USART_CR1_UE | USART_CR1_RE,
        0,
        0
    };

    // Configure the port
    if(port == 1) {
        palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT);
        esc_telem.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT);
        esc_telem.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_RX_LINE, PAL_MODE_INPUT);
        esc_telem.port = &UARTD3;
    } else{
        esc_telem.port = NULL;
    }

    // Open the telemetry port and start the thread
    if(esc_telem.port != NULL) {
        uartStart(esc_telem.port, &uart_cfg);
        chThdCreateStatic(esc_telem_wa, sizeof(esc_telem_wa), NORMALPRIO-5, esc_telem_thd, NULL);
        chThdCreateStatic(esc_telem_send_wa, sizeof(esc_telem_send_wa), NORMALPRIO-6, esc_telem_send_thd, NULL);
    }
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}

/**
 * T-Motor F.. protocol
*/
static void esc_telem_parse_tmotorf(uint8_t msg[], uint8_t len) { 
    // Verify the CRC and size
    if(len == 10 && get_crc8(msg, 9) == msg[9]) {
        uint16_t volt_u = (msg[1] << 8) | msg[2];
        uint16_t curr_u = (msg[3] << 8) | msg[4];
        uint16_t erpm_u = (msg[7] << 8) | msg[8];

        esc_telem.data.temp = (int8_t)msg[0];
        esc_telem.data.voltage = volt_u / 100.0f;
        esc_telem.data.current = curr_u / 100.0f;
        esc_telem.data.consumption = (msg[5] << 8) | msg[6];
        esc_telem.data.erpm = erpm_u * 100;
        esc_telem.data.received = true;
    }
}

static const uint8_t tempTable[ 220 ] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,16,17,18,19,19,20,21,21,22,23,23,24,25,25,26,26,27,28,28,29,29,30,30,
			  31,32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,40,40,41,41,42,42,43,43,44,44,44,45,45,46,46,47,47,48,48,49,49,50,50,50,51,51,52,52,53,53,
			  53,54,54,55,55,56,56,57,57,58,58,58,59,59,60,60,61,61,61,62,62,63,63,64,64,65,65,66,66,66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,74,74,75,
			  75,75,76,77,77,78,78,79,79,80,80,81,81,82,82,83,83,84,85,85,86,86,86,87,88,88,89,90,90,91,92,92,93,94,95,95,96,96,96,97,98,98,99,100,101,101,
			  102,103,103,104,105,106,106,107,107,108,109,110,110,111,112,113,113,114,115,115,116,117,117,118,119,120,120,121,122,122,123,124,125,125,126,
			  127,127,128,129,129};


/**
 * T-Motor Flame protocol
*/
static void esc_telem_parse_tmotor_flame(uint8_t msg[], uint8_t len)
{
    // Verify the start
    if(len < 24 || msg[0] != 0x9B || msg[1] != 0x16 || msg[2] != 0x04 || msg[3] != 0x02)
        return;

    // Very the checksum
    uint8_t checksum = 0;
    for(uint8_t i = 0; i < 22; i++)
        checksum += msg[i];
    uint8_t recv_checksum = (uint16_t)((msg[22]) | (msg[23]<<8));
    if(checksum != recv_checksum)
        return;

    // Dummy variables to apply the bitshifting later (for scaling according to protocol)
    // uint16_t rx_throttle_dummy = (msg[6] <<8 | msg[7]);
    // uint16_t output_throttle_dummy = (msg[8] <<8 | msg[9]);
    uint16_t rpm_dummy = (msg[10] <<8 | msg[11]);

    // Read the alpha esc information
    // alpha_esc_data.bale_no = (uint16_t)((msg[4] <<8 | msg[5]));
    // alpha_esc_data.rx_throttle = (uint16_t)((rx_throttle_dummy)*100/1024); // Original (uint16_t)((msg[6] <<8 | msg[7])*100/1024);
    // alpha_esc_data.output_throttle = (uint16_t)((output_throttle_dummy)*100/1024); // Original (uint16_t)((msg[8] <<8 | msg[9])*100/1024);
    esc_telem.data.erpm = (uint16_t)(rpm_dummy) * 500; // (uint16_t)((msg[10] <<8 | msg[11])*10/108);
    esc_telem.data.voltage = ((uint16_t)((msg[12] <<8 | msg[13]))) / 59.f; // Needs to be divided by 10 to get real voltage
    esc_telem.data.current = ((int16_t)((msg[14] <<8 | msg[15]))) / 7.f;// Needs to be divided by 64
    if(esc_telem.data.current > 0)
        esc_telem.data.current -= 48;
    // alpha_esc_data.phase_wire_current = ((int16_t)((msg[16] <<8 | msg[17]))); //Needs to be divided by 64
    esc_telem.data.temp = tempTable[msg[18]];
    // alpha_esc_data.capacitor_temp = tempTable[msg[19]];
    // alpha_esc_data.status_code = (uint16_t)((msg[20] <<8 | msg[21]));
    // alpha_esc_data.verify_code = (uint16_t)((msg[22] <<8 | msg[23]));// shift the right byte instead of the

    esc_telem.data.received = true;
}

/**
 * T-Motor Alpha protocol
*/
static void esc_telem_parse_tmotor_alpha(uint8_t msg[], uint8_t len)
{
    // Verify the start
    if(len < 24 || msg[0] != 0x9B || msg[1] != 0x16 || msg[2] != 0x01 || msg[3] != 0x02)
        return;

    // Very the checksum
    uint8_t checksum = 0;
    for(uint8_t i = 0; i < 22; i++)
        checksum += msg[i];
    uint8_t recv_checksum = (uint16_t)((msg[22]) | (msg[23]<<8));
    if(checksum != recv_checksum)
        return;

    // Dummy variables to apply the bitshifting later (for scaling according to protocol)
    //uint16_t rx_throttle_dummy = (msg[6] <<8 | msg[7]);
    //uint16_t output_throttle_dummy = (msg[8] <<8 | msg[9]);
    uint16_t rpm_dummy = (msg[10] <<8 | msg[11]);
    // Read the alpha esc information
    //alpha_esc_data.bale_no = (uint16_t)((msg[4] <<8 | msg[5]));
    //alpha_esc_data.rx_throttle = (uint16_t)((rx_throttle_dummy)*100/1024); // Original (uint16_t)((msg[6] <<8 | msg[7])*100/1024);
    //alpha_esc_data.output_throttle = (uint16_t)((output_throttle_dummy)*100/1024); // Original (uint16_t)((msg[8] <<8 | msg[9])*100/1024);
    esc_telem.data.erpm = (uint16_t)(rpm_dummy) * 20;
    esc_telem.data.voltage = ((uint16_t)((msg[12] <<8 | msg[13]))) / 10.0f; // Needs to be divided by 10 to get real voltage
    esc_telem.data.current = ((int16_t)((msg[14] <<8 | msg[15]))) / 64.f;// Needs to be divided by 64
    //alpha_esc_data.phase_wire_current = ((int16_t)((msg[16] <<8 | msg[17]))); //Needs to be divided by 64
    esc_telem.data.temp = tempTable[msg[18]];
    //alpha_esc_data.capacitor_temp = Table[msg[19]];
    //alpha_esc_data.status_code = (uint16_t)((msg[20] <<8 | msg[21]));
    //alpha_esc_data.verify_code = (uint16_t)((msg[22] <<8 | msg[23]));// shift the right byte instead of the

    esc_telem.data.received = true;
}