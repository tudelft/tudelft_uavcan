#include "tfmini.h"
#include "config.h"
#include <math.h>

struct tfmini_t tfmini;

/**
 * Parse the lidar bytes 1 by 1
 */
static void tfmini_parse(uint8_t byte)
{
  switch (tfmini.parse_status) {
    case TFMINI_PARSE_IDLE:
      if (byte == 0x59) {
        tfmini.parse_crc = byte;
        tfmini.parse_status++;
      }
      break;
    case TFMINI_PARSE_HEAD:
      if (byte == 0x59) {
        tfmini.parse_crc += byte;
        tfmini.parse_status++;
      } else {
        tfmini.parse_status = TFMINI_PARSE_IDLE;
      }
      break;

    case TFMINI_PARSE_DIST_L:
      tfmini.raw_dist = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_DIST_H:
      tfmini.raw_dist |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_STRENGTH_L:
      tfmini.raw_strength = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_STRENGTH_H:
      tfmini.raw_strength |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_TEMP_L:
      tfmini.raw_temp = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_TEMP_H:
      tfmini.raw_temp |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_CHECKSUM:
      // When the CRC matches
      if (tfmini.parse_crc == byte) {
        float distance = tfmini.raw_dist / 100.f;
        //float temp = tfmini.raw_temp * 10.f / 8.f - 256.f;

        // Set the values
        struct uavcan_equipment_range_sensor_Measurement measurement;
        measurement.timestamp.usec = TIME_I2US(chVTGetSystemTimeX());
        measurement.sensor_id = 0;
        measurement.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[0] = 0;
        measurement.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1] = 0;
        measurement.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[2] = 0;
        measurement.field_of_view = 2.f / 180 * M_PI; // 2 degrees
        measurement.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
        
        // Check if the measurement is valid
        if (tfmini.raw_dist == 65535) {
          measurement.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
        } else if(tfmini.raw_dist == 65534) {
          measurement.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
        } else if(tfmini.raw_dist == 65532) {
          measurement.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
        } else {
          measurement.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
        }
        measurement.range = distance;

        // Broadcast the message
        uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&measurement, buffer);
        static uint8_t transfer_id;
        uavcanBroadcastAll(
            UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
            UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID, &transfer_id,
            CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
      }

      // Start reading again
      tfmini.parse_status = TFMINI_PARSE_IDLE;
      break;

    default:
      // Error, return to start
      tfmini.parse_status = TFMINI_PARSE_IDLE;
      break;
  }
}

static THD_WORKING_AREA(tfmini_wa, 512);
static THD_FUNCTION(tfmini_thd, arg) {
  (void)arg;
  chRegSetThreadName("tfmini");

  while (true) {
    // Try to receive bytes and parse them
    size_t recv_size = 8;
    uint8_t buf[8];
    if(uartReceiveTimeout(tfmini.port, &recv_size, (void *)buf, TIME_MS2I(5)) != MSG_RESET) {
        for(uint16_t i = 0; i < recv_size; i++) {
            tfmini_parse(buf[i]);
        }
    }

  }
}

void tfmini_init(void) {
    // Get the configuration
    uint32_t baudrate = config_get_by_name("TFMINI baudrate", 0)->val.i;
    tfmini.frequency = config_get_by_name("TFMINI frequency", 0)->val.f;

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
    uint8_t port = config_get_by_name("TFMINI port", 0)->val.i;
    if(port == 1) {
        palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL1_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        tfmini.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        tfmini.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL3_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        tfmini.port = &UARTD3;
    } else{
        tfmini.port = NULL;
    }

    // Open the comminication port and start the thread
    if(tfmini.port != NULL) {
        tfmini.parse_status = TFMINI_PARSE_IDLE;
        uartStart(tfmini.port, &uart_cfg);
        chThdCreateStatic(tfmini_wa, sizeof(tfmini_wa), NORMALPRIO-10, tfmini_thd, NULL);
    }
}