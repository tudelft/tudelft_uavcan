#include "drs_parachute.h"
#include "config.h"
#include <ardupilotmega/mavlink.h>


struct drs_parachute_t drs_parachute;

/**
 * Transmit a mavlink message over the UART port
*/
static void mavlink_send(mavlink_message_t *msg) {
    uint8_t buf[300];
    size_t len = mavlink_msg_to_send_buffer(buf, msg);
    uartSendFullTimeout(drs_parachute.port, &len, (void *)buf, TIME_MS2I(100));
}

/* Send heartbeat */
static void mavlink_send_heartbeat(void) {
    mavlink_heartbeat_t heartbeat;
    heartbeat.type = MAV_TYPE_GENERIC;
    heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_STANDBY;

    mavlink_message_t message;
    mavlink_msg_heartbeat_encode(11, MAV_COMP_ID_AUTOPILOT1, &message, &heartbeat);
    mavlink_send(&message);
}

/* Send autopilot version */
static void mavlink_send_autopilot_version(void) {
    mavlink_autopilot_version_t autopilot_version;
    autopilot_version.capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    autopilot_version.flight_sw_version = SOFT_VER_MAJOR * 100 + SOFT_VER_MINOR;

    mavlink_message_t message;
    mavlink_msg_autopilot_version_encode(11, MAV_COMP_ID_AUTOPILOT1, &message, &autopilot_version);
    mavlink_send(&message);
}

/* Send deploy parachute */
static void mavlink_send_cmd_parachute(uint8_t action) {
    mavlink_command_long_t command_long;
    command_long.target_system = 11;
    command_long.target_component = MAV_COMP_ID_PARACHUTE;
    command_long.command = MAV_CMD_DO_PARACHUTE;
    command_long.param1 = action;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(11, MAV_COMP_ID_AUTOPILOT1, &message, &command_long);
    mavlink_send(&message);
}

/* Send command ack message */
static void mavlink_send_command_ack(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component) {
    mavlink_command_ack_t command_ack;
    command_ack.command = command;
    command_ack.result = result;
    command_ack.target_system = target_system;
    command_ack.target_component = target_component;

    mavlink_message_t message;
    mavlink_msg_command_ack_encode(11, MAV_COMP_ID_AUTOPILOT1, &message, &command_ack);
    mavlink_send(&message);
}

static THD_WORKING_AREA(drs_parachute_wa, 3072);
static THD_FUNCTION(drs_parachute_thd, arg) {
  (void)arg;
  chRegSetThreadName("drs_parachute");
  mavlink_message_t message;
  mavlink_status_t status;

  // Wait for bootup
  systime_t last_heartbeat = 0;
  while (true) {
    // Send heartbeat every 500ms
    if(chVTTimeElapsedSinceX(last_heartbeat) > TIME_MS2I(500)) {
        mavlink_send_heartbeat();
        last_heartbeat = chVTGetSystemTimeX();
    }
    
    // Try to receive bytes
    size_t recv_size = 50;
    uint8_t buf[50];
    if(uartReceiveTimeout(drs_parachute.port, &recv_size, (void *)buf, TIME_MS2I(20)) != MSG_RESET) {
        for(uint16_t i = 0; i < recv_size; i++) {
            // Received a message
            if(mavlink_parse_char(MAVLINK_COMM_1, buf[i], &message, &status)) {
                switch(message.msgid) {
                    case MAVLINK_MSG_ID_COMMAND_LONG:
                    {
                        mavlink_command_long_t command_long;
                        mavlink_msg_command_long_decode(&message, &command_long);

                        if(command_long.command == MAV_CMD_REQUEST_MESSAGE && command_long.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                            mavlink_send_autopilot_version();
                            chThdSleepMilliseconds(100);
                        }
                        else if(command_long.command == MAV_CMD_DO_FLIGHTTERMINATION && command_long.param1 > 0.5) {
                            // TODO: kill all the motors
                            mavlink_send_command_ack(MAV_CMD_DO_FLIGHTTERMINATION, MAV_RESULT_ACCEPTED, message.sysid, message.compid);
                            chThdSleepMilliseconds(50);
                        }
                        else if(command_long.command == MAV_CMD_COMPONENT_ARM_DISARM && command_long.param1 == 0  && command_long.param2 == 21196) {
                            // TODO: kill all the motors
                            mavlink_send_command_ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, message.sysid, message.compid);
                            chThdSleepMilliseconds(50);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        static uint16_t heartbeat_cnt = 0;
                        if(drs_parachute.status == DRS_STATUS_INIT && heartbeat_cnt > 3) {
                            mavlink_send_cmd_parachute(PARACHUTE_DISABLE);
                            drs_parachute.status = DRS_STATUS_DISABLE;
                            chThdSleepMilliseconds(100);
                        }
                        heartbeat_cnt++;
                        break;
                    }
                    default:
                        break;
                }
            }

        }
    }
  }
}

/* Do actions on the parachute */
void drs_parachute_set(enum parachute_status_t status) {
    static systime_t last_cmd = 0;
    // Don't send if initializing or too fast
    if(drs_parachute.status == DRS_STATUS_INIT || chVTTimeElapsedSinceX(last_cmd) < TIME_MS2I(100))
        return;

    // Don't resend arm/disarm
    if(drs_parachute.status == status && status != DRS_STATUS_RELEASE)
        return;
    
    // Send the command
    drs_parachute.status = status;
    last_cmd = chVTGetSystemTimeX();

    if(status == DRS_STATUS_RELEASE) {
        mavlink_send_cmd_parachute(PARACHUTE_RELEASE);
    } else if (status == DRS_STATUS_DISABLE) {
        mavlink_send_cmd_parachute(PARACHUTE_DISABLE);
    } else if (status == DRS_STATUS_ENABLE) {
        mavlink_send_cmd_parachute(PARACHUTE_ENABLE);
    }
}

void drs_parachute_init(void) {
    // Get the configuration
    drs_parachute.index = config_get_by_name("DRS index", 0)->val.i;
    uint32_t baudrate = config_get_by_name("DRS baudrate", 0)->val.i;
    

    // The UART port settings
    UARTConfig uart_cfg = {
        NULL,
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
    uint8_t port = config_get_by_name("DRS port", 0)->val.i;
    if(port == 1) {
        palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL1_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        drs_parachute.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        drs_parachute.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL3_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        drs_parachute.port = &UARTD3;
    } else{
        drs_parachute.port = NULL;
    }

    // Open the telemetry port and start the thread
    if(drs_parachute.port != NULL) {
        uartStart(drs_parachute.port, &uart_cfg);
        chThdCreateStatic(drs_parachute_wa, sizeof(drs_parachute_wa), NORMALPRIO+3, drs_parachute_thd, NULL);
    }
}
