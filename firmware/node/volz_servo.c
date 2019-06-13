#include "volz_servo.h"

#include <ch.h>
#include <hal.h>
#include "config.h"

static void end_cb(UARTDriver *uartp) {
    palClearLine(RS485_DE_LINE);
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
  NULL,
  end_cb,
  NULL,
  NULL,
  NULL,
  115200,
  0,
  USART_CR2_LINEN,
  0
};

static uint8_t p_gain = 75;
static uint8_t d_gain = 40;
static uint8_t max_power = 80;
static uint8_t stall_power = 45;

static void calc_crc(uint8_t *data) {
  uint16_t crc = 0xFFFF;
  for(uint8_t i = 0; i < 4; i++) {
    crc = ((data[i] << 8) ^ crc);

    for(uint8_t j = 0; j < 8; j++) {
      if(crc & 0x8000)
        crc = (crc << 1) ^ 0x8005;
      else
        crc = crc << 1;
    }
  }
  data[4] = crc >> 8;
  data[5] = crc & 0xFF;
}

static void write_cmd(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t *resp) {
  uint8_t resp_data[6];
  uint8_t data[6] = {cmd, 0x01, arg1, arg2, 0x00, 0x00};
  calc_crc(data);

  palSetLine(RS485_DE_LINE);
  uartStartSend(&UARTD3, 6, data);

  if(resp == NULL)
    resp = resp_data;

  size_t recv_size = 6;
  uartReceiveTimeout(&UARTD3, &recv_size, (void *)resp, TIME_MS2I(2));
}

static void write_eeprom(uint8_t addr, uint8_t value) {
  write_cmd(0xE8, addr, value, NULL);
}

void volz_servo_init(void) {
  uartStart(&UARTD3, &uart_cfg_1);
  palClearLine(RS485_DE_LINE);
  palClearLine(RS485_RE_LINE);

  p_gain = config_get_by_name("SERVO P-gain", 0)->val.i;
  d_gain = config_get_by_name("SERVO D-gain", 0)->val.i;
  max_power = config_get_by_name("SERVO max-power", 0)->val.i;
  stall_power = config_get_by_name("SERVO stall-power", 0)->val.i;

  write_eeprom(0x20, d_gain);
  write_eeprom(0x21, p_gain);
  write_eeprom(0x24, max_power);
  write_eeprom(0x25, stall_power);
}

int16_t last_val = 0;
void volz_servo_set(int16_t val) {
  if(val != last_val) {
    uint16_t cmd = 0x0800 + ((float)val / 8191.0 * 0x360 );
    write_cmd(0xDD, (cmd >> 7)&0x1F, cmd&0x7F, NULL);
    last_val = val;
  }
}