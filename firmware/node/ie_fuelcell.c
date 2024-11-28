#include "ie_fuelcell.h"

#include "config.h"
#include <stdlib.h>

struct ie_fuelcell_t ie_fuelcell = {0};

static THD_WORKING_AREA(ie_fuelcell_wa, 512);
static THD_FUNCTION(ie_fuelcell_thd, arg) {
  (void)arg;
  chRegSetThreadName("ie_fuelcell");
  while (true) {
    

    // Try to receive bytes
    char buf[256];
    size_t recv_size = 256;
    if(uartReceiveTimeout(ie_fuelcell.port, &recv_size, (void *)&buf, TIME_MS2I(10)) != MSG_RESET) {
        // Go through all characters
        for(uint16_t i = 0; i < recv_size; i++) {
            // If we receive a start reset
            if(buf[i] == '<') {
                ie_fuelcell.data.term_id = 0;
                ie_fuelcell.data.term_size = 0;
                ie_fuelcell.data.calc_checksum = '<';
                ie_fuelcell.data.in_string = true;
            }
            // Ignore other strings
            else if(!ie_fuelcell.data.in_string) {
                continue;
            }
            // The END
            else if(buf[i] == '>') {
                ie_fuelcell.data.in_string = false;
                ie_fuelcell.data.term[ie_fuelcell.data.term_size++] = 0;
                ie_fuelcell.data.checksum = ~strtoul(ie_fuelcell.data.term, NULL, 10);

                if(ie_fuelcell.data.term_id == 11 && ie_fuelcell.data.checksum == ie_fuelcell.data.calc_checksum)
                    ie_fuelcell.data.received = true;
            }
            // Parse the term
            else if(buf[i] == ',') {
                ie_fuelcell.data.calc_checksum += ',';
                ie_fuelcell.data.term[ie_fuelcell.data.term_size++] = 0;
                switch(ie_fuelcell.data.term_id) {
                    case 0:
                        ie_fuelcell.data.tank_pressure = strtol(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 1:
                        ie_fuelcell.data.regulated_pressure = strtof(ie_fuelcell.data.term, NULL) * 100.f;
                        break;
                    case 2:
                        ie_fuelcell.data.battery_voltage = strtof(ie_fuelcell.data.term, NULL) * 10.f;
                        break;
                    case 3:
                        ie_fuelcell.data.output_power = strtol(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 4:
                        ie_fuelcell.data.spm_power = strtoul(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 5:
                        // Ignore unit fault
                        break;
                    case 6:
                        ie_fuelcell.data.battery_power = strtol(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 7:
                        ie_fuelcell.data.psu_state = strtoul(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 8:
                        ie_fuelcell.data.error_code = strtoul(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 9:
                        ie_fuelcell.data.sub_code = strtoul(ie_fuelcell.data.term, NULL, 10);
                        break;
                    case 10:
                        // Ignore string
                        break;
                }
                ie_fuelcell.data.term_id++;
                ie_fuelcell.data.term_size = 0;
            }
            // Copy the term
            else {
                ie_fuelcell.data.term[ie_fuelcell.data.term_size++] = buf[i];
                if(ie_fuelcell.data.term_id <= 10)
                    ie_fuelcell.data.calc_checksum += buf[i];
            }
        }
    }
  }
}

static void ie_fuelcell_broadcast(void) {
  // Inhibit when no telemetry is received
  if(!ie_fuelcell.data.received) {
    ie_fuelcell.data.timeout_cnt++;
    return;
  }

  // Set the values
  struct uavcan_equipment_fuelcell_Status fuelcellStatus;
  fuelcellStatus.tank_pressure = ie_fuelcell.data.tank_pressure;
  fuelcellStatus.regulated_pressure = ie_fuelcell.data.regulated_pressure;
  fuelcellStatus.battery_voltage = ie_fuelcell.data.battery_voltage;
  fuelcellStatus.output_power = ie_fuelcell.data.output_power;
  fuelcellStatus.spm_power = ie_fuelcell.data.spm_power;
  fuelcellStatus.battery_power = ie_fuelcell.data.battery_power;
  fuelcellStatus.psu_state = ie_fuelcell.data.psu_state;
  fuelcellStatus.error_code = ie_fuelcell.data.error_code;
  fuelcellStatus.sub_code = ie_fuelcell.data.sub_code;

  uint8_t buffer[UAVCAN_EQUIPMENT_FUELCELL_STATUS_MAX_SIZE];
  uint16_t total_size = uavcan_equipment_fuelcell_Status_encode(&fuelcellStatus, buffer);

  static uint8_t transfer_id;
  uavcanBroadcastAll(
      UAVCAN_EQUIPMENT_FUELCELL_STATUS_SIGNATURE,
      UAVCAN_EQUIPMENT_FUELCELL_STATUS_ID, &transfer_id,
      CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
  ie_fuelcell.data.received = false;
}

static THD_WORKING_AREA(ie_fuelcell_send_wa, 512);
static THD_FUNCTION(ie_fuelcell_send_thd, arg) {
  (void)arg;
  chRegSetThreadName("ie_fuelcell_send");
  while (true) {
    ie_fuelcell_broadcast();
    chThdSleepMilliseconds(ie_fuelcell.vt_delay);
  }
}


void ie_fuelcell_init(void) {
    // Get the configuration
    ie_fuelcell.vt_delay = 1000.f / config_get_by_name("IE FC frequency", 0)->val.f;
    uint8_t port = config_get_by_name("IE FC port", 0)->val.i;
    uint32_t baud = config_get_by_name("IE FC baud", 0)->val.i;

    // Change port settings based on type
    UARTConfig uart_cfg = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        baud,
        USART_CR1_UE | USART_CR1_RE,
        0,
        0
    };

    // Configure the port
    if(port == 1) {
        palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT);
        ie_fuelcell.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT);
        ie_fuelcell.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_RX_LINE, PAL_MODE_INPUT);
        ie_fuelcell.port = &UARTD3;
    } else{
        ie_fuelcell.port = NULL;
    }

    // Open the telemetry port and start the thread
    if(ie_fuelcell.port != NULL) {
        uartStart(ie_fuelcell.port, &uart_cfg);
        chThdCreateStatic(ie_fuelcell_wa, sizeof(ie_fuelcell_wa), NORMALPRIO-5, ie_fuelcell_thd, NULL);
        chThdCreateStatic(ie_fuelcell_send_wa, sizeof(ie_fuelcell_send_wa), NORMALPRIO-6, ie_fuelcell_send_thd, NULL);
    }
}
