#ifndef TFMINI_H
#define TFMINI_H

#include <hal.h>

enum tfmini_parse_status_t {
  TFMINI_PARSE_IDLE,
  TFMINI_PARSE_HEAD,
  TFMINI_PARSE_DIST_L,
  TFMINI_PARSE_DIST_H,
  TFMINI_PARSE_STRENGTH_L,
  TFMINI_PARSE_STRENGTH_H,
  TFMINI_PARSE_TEMP_L,
  TFMINI_PARSE_TEMP_H,
  TFMINI_PARSE_CHECKSUM
};


struct tfmini_t {
    UARTDriver *port;                   ///< Serial port
    float frequency;                    ///< The transmitting frequency
    UARTConfig uart_cfg;                ///< UART configuration

    enum tfmini_parse_status_t parse_status;    ///< Parser state
    uint8_t parse_crc;                          ///< Calculated CRC
    uint16_t raw_dist;                          ///< Raw distance
    uint16_t raw_strength;                      ///< Raw strength
    uint8_t raw_temp;                           ///< Raw temperature
};
extern struct tfmini_t tfmini;

void tfmini_init(void);

#endif /* TFMINI_H */