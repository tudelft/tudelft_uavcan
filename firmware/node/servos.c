#include "servos.h"

#include "config.h"

#include <ch.h>
#include <hal.h>
#include <string.h>

struct servos_t {
  bool initialized;
  virtual_timer_t timeout_vt;
  uint32_t node_timeout;

#ifdef SERVO1_LINE
  uint8_t servo1_idx;
  uint16_t servo1_failsafe;
  bool servo1_tmotor;
#endif
#ifdef SERVO2_LINE
  uint8_t servo2_idx;
  uint16_t servo2_failsafe;
  bool servo2_tmotor;
#endif
#ifdef SERVO3_LINE
  uint8_t servo3_idx;
  uint16_t servo3_failsafe;
  bool servo3_tmotor;
#endif
#ifdef SERVO4_LINE
  uint8_t servo4_idx;
  uint16_t servo4_failsafe;
  bool servo4_tmotor;
#endif
#ifdef SERVO5_LINE
  uint8_t servo5_idx;
  uint16_t servo5_failsafe;
  bool servo5_tmotor;
#endif
#ifdef SERVO6_LINE
  uint8_t servo6_idx;
  uint16_t servo6_failsafe;
  bool servo6_tmotor;
#endif
#ifdef SERVO7_LINE
  uint8_t servo7_idx;
  uint16_t servo7_failsafe;
  bool servo7_tmotor;
#endif
#ifdef SERVO8_LINE
  uint8_t servo8_idx;
  uint16_t servo8_failsafe;
  bool servo8_tmotor;
#endif
#ifdef SERVO9_LINE
  uint8_t servo9_idx;
  uint16_t servo9_failsafe;
  bool servo9_tmotor;
#endif
#ifdef SERVO10_LINE
  uint8_t servo10_idx;
  uint16_t servo10_failsafe;
  bool servo10_tmotor;
#endif
};
static struct servos_t servos = {
  .initialized = false
};

static void servos_timeout_cb(virtual_timer_t *vtp __attribute__((unused)), void *p __attribute__((unused))) {
  // When no commands are received timeout and set everything to failsafe
  uint16_t servo_values[] = {
#ifdef SERVO1_LINE
    servos.servo1_failsafe,
#endif
#ifdef SERVO2_LINE
    servos.servo2_failsafe,
#endif
#ifdef SERVO3_LINE
    servos.servo3_failsafe,
#endif
#ifdef SERVO4_LINE
    servos.servo4_failsafe,
#endif
#ifdef SERVO5_LINE
    servos.servo5_failsafe,
#endif
#ifdef SERVO6_LINE
    servos.servo6_failsafe,
#endif
#ifdef SERVO7_LINE
    servos.servo7_failsafe,
#endif
#ifdef SERVO8_LINE
    servos.servo8_failsafe,
#endif
#ifdef SERVO9_LINE
    servos.servo9_failsafe,
#endif
#ifdef SERVO10_LINE
    servos.servo10_failsafe,
#endif
  };
  board_set_servos(false, servo_values, sizeof(servo_values) / sizeof(uint16_t));
}

void servos_init(void) {
  servos.node_timeout = config_get_by_name("SERVO failsafe timeout (ms)", 0)->val.i;

  // Read the servo settings
#ifdef SERVO1_LINE
  servos.servo1_idx = config_get_by_name("SERVO1 index", 0)->val.i;
  servos.servo1_failsafe = config_get_by_name("SERVO1 failsafe", 0)->val.i;
  if(servos.servo1_idx >= 100 && servos.servo1_idx != 255) {
    servos.servo1_idx -= 100;
    servos.servo1_tmotor = true;
  }
#endif
#ifdef SERVO2_LINE
  servos.servo2_idx = config_get_by_name("SERVO2 index", 0)->val.i;
  servos.servo2_failsafe = config_get_by_name("SERVO2 failsafe", 0)->val.i;
  if(servos.servo2_idx >= 100 && servos.servo2_idx != 255) {
    servos.servo2_idx -= 100;
    servos.servo2_tmotor = true;
  }
#endif
#ifdef SERVO3_LINE
  servos.servo3_idx = config_get_by_name("SERVO3 index", 0)->val.i;
  servos.servo3_failsafe = config_get_by_name("SERVO3 failsafe", 0)->val.i;
  if(servos.servo3_idx >= 100 && servos.servo3_idx != 255) {
    servos.servo3_idx -= 100;
    servos.servo3_tmotor = true;
  }
#endif
#ifdef SERVO4_LINE
  servos.servo4_idx = config_get_by_name("SERVO4 index", 0)->val.i;
  servos.servo4_failsafe = config_get_by_name("SERVO4 failsafe", 0)->val.i;
  if(servos.servo4_idx >= 100 && servos.servo4_idx != 255) {
    servos.servo4_idx -= 100;
    servos.servo4_tmotor = true;
  }
#endif
#ifdef SERVO5_LINE
  servos.servo5_idx = config_get_by_name("SERVO5 index", 0)->val.i;
  servos.servo5_failsafe = config_get_by_name("SERVO5 failsafe", 0)->val.i;
  if(servos.servo5_idx >= 100 && servos.servo5_idx != 255) {
    servos.servo5_idx -= 100;
    servos.servo5_tmotor = true;
  }
#endif
#ifdef SERVO6_LINE
  servos.servo6_idx = config_get_by_name("SERVO6 index", 0)->val.i;
  servos.servo6_failsafe = config_get_by_name("SERVO6 failsafe", 0)->val.i;
  if(servos.servo6_idx >= 100 && servos.servo6_idx != 255) {
    servos.servo6_idx -= 100;
    servos.servo6_tmotor = true;
  }
#endif
#ifdef SERVO7_LINE
  servos.servo7_idx = config_get_by_name("SERVO7 index", 0)->val.i;
  servos.servo7_failsafe = config_get_by_name("SERVO7 failsafe", 0)->val.i;
  if(servos.servo7_idx >= 100 && servos.servo7_idx != 255) {
    servos.servo7_idx -= 100;
    servos.servo7_tmotor = true;
  }
#endif
#ifdef SERVO8_LINE
  servos.servo8_idx = config_get_by_name("SERVO8 index", 0)->val.i;
  servos.servo8_failsafe = config_get_by_name("SERVO8 failsafe", 0)->val.i;
  if(servos.servo8_idx >= 100 && servos.servo8_idx != 255) {
    servos.servo8_idx -= 100;
    servos.servo8_tmotor = true;
  }
#endif
#ifdef SERVO9_LINE
  servos.servo9_idx = config_get_by_name("SERVO9 index", 0)->val.i;
  servos.servo9_failsafe = config_get_by_name("SERVO9 failsafe", 0)->val.i;
  if(servos.servo9_idx >= 100 && servos.servo9_idx != 255) {
    servos.servo9_idx -= 100;
    servos.servo9_tmotor = true;
  }
#endif
#ifdef SERVO10_LINE
  servos.servo10_idx = config_get_by_name("SERVO10 index", 0)->val.i;
  servos.servo10_failsafe = config_get_by_name("SERVO10 failsafe", 0)->val.i;
  if(servos.servo10_idx >= 100 && servos.servo10_idx != 255) {
    servos.servo10_idx -= 100;
    servos.servo10_tmotor = true;
  }
#endif

  // Initialize the servos which are used
  bool servos_enable[] = {
#ifdef SERVO1_LINE
    (servos.servo1_idx != 255),
#endif
#ifdef SERVO2_LINE
    (servos.servo2_idx != 255),
#endif
#ifdef SERVO3_LINE
    (servos.servo3_idx != 255),
#endif
#ifdef SERVO4_LINE
    (servos.servo4_idx != 255),
#endif
#ifdef SERVO5_LINE
    (servos.servo5_idx != 255),
#endif
#ifdef SERVO6_LINE
    (servos.servo6_idx != 255),
#endif
#ifdef SERVO7_LINE
    (servos.servo7_idx != 255),
#endif
#ifdef SERVO8_LINE
    (servos.servo8_idx != 255),
#endif
#ifdef SERVO9_LINE
    (servos.servo9_idx != 255),
#endif
#ifdef SERVO10_LINE
    (servos.servo10_idx != 255),
#endif
  };
  board_init_servos(servos_enable, sizeof(servos_enable) / sizeof(bool));

  // Set all servos to the initial failsafe value
  uint16_t servo_values[] = {
#ifdef SERVO1_LINE
    servos.servo1_failsafe,
#endif
#ifdef SERVO2_LINE
    servos.servo2_failsafe,
#endif
#ifdef SERVO3_LINE
    servos.servo3_failsafe,
#endif
#ifdef SERVO4_LINE
    servos.servo4_failsafe,
#endif
#ifdef SERVO5_LINE
    servos.servo5_failsafe,
#endif
#ifdef SERVO6_LINE
    servos.servo6_failsafe,
#endif
#ifdef SERVO7_LINE
    servos.servo7_failsafe,
#endif
#ifdef SERVO8_LINE
    servos.servo8_failsafe,
#endif
#ifdef SERVO9_LINE
    servos.servo9_failsafe,
#endif
#ifdef SERVO10_LINE
    servos.servo10_failsafe,
#endif
  };
  board_set_servos(true, servo_values, sizeof(servo_values) / sizeof(uint16_t));

  // Initialize the servo timeout timer
  chVTObjectInit(&servos.timeout_vt);
  servos.initialized = true;
}

void servos_disable(void) {
  servos.initialized = false;
  board_disable_servos();
}

uint16_t servo_cmd(uint8_t idx, struct uavcan_equipment_esc_RawCommand *msg, uint16_t failsafe, bool tmotor) {
  if(idx < msg->cmd.len) {
    int16_t servo_cmd = 1500+(msg->cmd.data[idx]*1000/8191);
    if(servo_cmd < 0) servo_cmd = 0;

    if(tmotor) {
      // Scale the command to the motor range
      servo_cmd = 1000 + (msg->cmd.data[idx]*1000/8191);
      if(servo_cmd < 1000) servo_cmd = 1000;
    }

    return servo_cmd;
  } else {
    return failsafe;
  }
}

/*
  handle a RAW_COMMAND request
 */
void handle_esc_rawcommand(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
  if(!servos.initialized)
    return;
  
  // Decode the commands
  struct uavcan_equipment_esc_RawCommand msg;
  if(uavcan_equipment_esc_RawCommand_decode(transfer, &msg))
    return;

  // Set the target commands for the servo
#ifdef SERVO1_LINE
  int16_t servo1_cmd = servo_cmd(servos.servo1_idx, &msg, servos.servo1_failsafe, servos.servo1_tmotor);
#endif
#ifdef SERVO2_LINE
  int16_t servo2_cmd = servo_cmd(servos.servo2_idx, &msg, servos.servo2_failsafe, servos.servo2_tmotor);
#endif
#ifdef SERVO3_LINE
  int16_t servo3_cmd = servo_cmd(servos.servo3_idx, &msg, servos.servo3_failsafe, servos.servo3_tmotor);
#endif
#ifdef SERVO4_LINE
  int16_t servo4_cmd = servo_cmd(servos.servo4_idx, &msg, servos.servo4_failsafe, servos.servo4_tmotor);
#endif
#ifdef SERVO5_LINE
  int16_t servo5_cmd = servo_cmd(servos.servo5_idx, &msg, servos.servo5_failsafe, servos.servo5_tmotor);
#endif
#ifdef SERVO6_LINE
  int16_t servo6_cmd = servo_cmd(servos.servo6_idx, &msg, servos.servo6_failsafe, servos.servo6_tmotor);
#endif
#ifdef SERVO7_LINE
  int16_t servo7_cmd = servo_cmd(servos.servo7_idx, &msg, servos.servo7_failsafe, servos.servo7_tmotor);
#endif
#ifdef SERVO8_LINE
  int16_t servo8_cmd = servo_cmd(servos.servo8_idx, &msg, servos.servo8_failsafe, servos.servo8_tmotor);
#endif
#ifdef SERVO9_LINE
  int16_t servo9_cmd = servo_cmd(servos.servo9_idx, &msg, servos.servo9_failsafe, servos.servo9_tmotor);
#endif
#ifdef SERVO10_LINE
  int16_t servo10_cmd = servo_cmd(servos.servo10_idx, &msg, servos.servo10_failsafe, servos.servo10_tmotor);
#endif

#include "faulhaber_ctrl.h"
  if(faulhaber_ctrl.port != NULL && faulhaber_ctrl.index < msg.cmd.len) {
    int64_t range = (faulhaber_ctrl.max_pos - faulhaber_ctrl.min_pos);
    faulhaber_ctrl.target_position = (uint64_t)faulhaber_ctrl.min_pos + ((msg.cmd.data[faulhaber_ctrl.index] + 8192)*range / (8191+8192));
  }

#include "drs_parachute.h"
  if(drs_parachute.port != NULL && drs_parachute.index < msg.cmd.len) {
    if(msg.cmd.data[drs_parachute.index] > 4000)
      drs_parachute_set(DRS_STATUS_RELEASE);
    else if(msg.cmd.data[drs_parachute.index] < -4000)
      drs_parachute_set(DRS_STATUS_DISABLE);
    else
      drs_parachute_set(DRS_STATUS_ENABLE);
  }

  // Commit the commands
  uint16_t servo_values[] = {
#ifdef SERVO1_LINE
    servo1_cmd,
#endif
#ifdef SERVO2_LINE
    servo2_cmd,
#endif
#ifdef SERVO3_LINE
    servo3_cmd,
#endif
#ifdef SERVO4_LINE
    servo4_cmd,
#endif
#ifdef SERVO5_LINE
    servo5_cmd,
#endif
#ifdef SERVO6_LINE
    servo6_cmd,
#endif
#ifdef SERVO7_LINE
    servo7_cmd,
#endif
#ifdef SERVO8_LINE
    servo8_cmd,
#endif
#ifdef SERVO9_LINE
    servo9_cmd,
#endif
#ifdef SERVO10_LINE
    servo10_cmd,
#endif
  };
  board_set_servos(true, servo_values, sizeof(servo_values) / sizeof(uint16_t));

  // Enable timeout
  chVTSet(&servos.timeout_vt, TIME_MS2I(servos.node_timeout), servos_timeout_cb, NULL);
}