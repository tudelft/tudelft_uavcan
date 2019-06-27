#include "volz_servo.h"

#include <ch.h>
#include <hal.h>
#include "config.h"

static void end_cb(UARTDriver *uartp __attribute__((unused))) {
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
static uint8_t servo_type = 0;

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

static void write_cmd(uint8_t cmd, uint8_t target, uint8_t arg1, uint8_t arg2, uint8_t *resp, uint8_t timeout_ms) {
  uint8_t resp_data[12];
  uint8_t data[6] = {cmd, target, arg1, arg2, 0x00, 0x00};
  calc_crc(data);

  palSetLine(RS485_DE_LINE);
  uartStartSend(&UARTD3, 6, data);

  if(resp == NULL)
    resp = resp_data;

  size_t recv_size = 12;
  uartReceiveTimeout(&UARTD3, &recv_size, (void *)resp, TIME_MS2I(timeout_ms));
}

static void write_eeprom(uint8_t addr, uint8_t value) {
  write_cmd(0xE8, 0x1F, addr, value, NULL, 15);
}

static void access_eeprom(void) {
  write_cmd(0xFF, 0x1F, 0x41, 0x3F, NULL, 15);
}

void volz_servo_init(void) {
  uartStart(&UARTD3, &uart_cfg_1);
  palClearLine(RS485_DE_LINE);
  palClearLine(RS485_RE_LINE);

  servo_type = config_get_by_name("SERVO type", 0)->val.i;
  p_gain = config_get_by_name("SERVO P-gain", 0)->val.i;
  d_gain = config_get_by_name("SERVO D-gain", 0)->val.i;
  max_power = config_get_by_name("SERVO max-power", 0)->val.i;
  stall_power = config_get_by_name("SERVO stall-power", 0)->val.i;

  /* Update servo settings */
  if(servo_type == 0) {
    access_eeprom();
    write_eeprom(0x20, d_gain);
    write_eeprom(0x21, p_gain);
    write_eeprom(0x24, max_power);
    write_eeprom(0x25, stall_power);
  }
}

int16_t last_val = -10000;
void volz_servo_set(int16_t val) {
  if(val != last_val && val < 8191 && val > -8191) {
    uint16_t cmd = 0x0800 + ((float)val / 8191.0 * 0x360 );
    write_cmd(0xDD, 0x01, (cmd >> 7)&0x1F, cmd&0x7F, NULL, 1);
    last_val = val;
  }
}