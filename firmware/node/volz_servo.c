#include "volz_servo.h"

#include <ch.h>
#include <hal.h>

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

void volz_servo_init(void) {
  uartStart(&UARTD3, &uart_cfg_1);
  palClearLine(RS485_DE_LINE);
  palClearLine(RS485_RE_LINE);

  uint8_t data[6] = {0xB4, 0x1F, 0x41, 0x53, 0x16, 0x72};
  palSetLine(RS485_DE_LINE);
  uartStartSend(&UARTD3, 6, data);
}

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

int16_t last_val = 0;
void volz_servo_set(int16_t val) {
    if(val != last_val) {
        uint16_t cmd = 0x0800 + ((float)val / 8191.0 * 0x360 );
        uint8_t data[6] = {0xDD, 0x01, (cmd >> 7)&0x1F, cmd&0x7F, 0x00, 0x00};
        calc_crc(data);

        palSetLine(RS485_DE_LINE);
        uartStartSend(&UARTD3, 6, data);
        last_val = val;
    }
}