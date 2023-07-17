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
#endif
#ifdef SERVO2_LINE
  uint8_t servo2_idx;
  uint16_t servo2_failsafe;
#endif
#ifdef SERVO3_LINE
  uint8_t servo3_idx;
  uint16_t servo3_failsafe;
#endif
#ifdef SERVO4_LINE
  uint8_t servo4_idx;
  uint16_t servo4_failsafe;
#endif
#ifdef SERVO5_LINE
  uint8_t servo5_idx;
  uint16_t servo5_failsafe;
#endif
#ifdef SERVO6_LINE
  uint8_t servo6_idx;
  uint16_t servo6_failsafe;
#endif
#ifdef SERVO7_LINE
  uint8_t servo7_idx;
  uint16_t servo7_failsafe;
#endif
};
static struct servos_t servos = {
  .initialized = false
};

static void servos_timeout_cb(void *arg __attribute__((unused))) {
  // When no commands are received timeout and set everything to failsafe
  board_set_servos(false,
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
    servos.servo7_failsafe
#endif
  );
}

void servos_init(void) {
  servos.node_timeout = config_get_by_name("SERVO failsafe timeout (ms)", 0)->val.i;

  // Read the servo settings
#ifdef SERVO1_LINE
  servos.servo1_idx = config_get_by_name("SERVO1 index", 0)->val.i;
  servos.servo1_failsafe = config_get_by_name("SERVO1 failsafe", 0)->val.i;
#endif
#ifdef SERVO2_LINE
  servos.servo2_idx = config_get_by_name("SERVO2 index", 0)->val.i;
  servos.servo2_failsafe = config_get_by_name("SERVO2 failsafe", 0)->val.i;
#endif
#ifdef SERVO3_LINE
  servos.servo3_idx = config_get_by_name("SERVO3 index", 0)->val.i;
  servos.servo3_failsafe = config_get_by_name("SERVO3 failsafe", 0)->val.i;
#endif
#ifdef SERVO4_LINE
  servos.servo4_idx = config_get_by_name("SERVO4 index", 0)->val.i;
  servos.servo4_failsafe = config_get_by_name("SERVO4 failsafe", 0)->val.i;
#endif
#ifdef SERVO5_LINE
  servos.servo5_idx = config_get_by_name("SERVO5 index", 0)->val.i;
  servos.servo5_failsafe = config_get_by_name("SERVO5 failsafe", 0)->val.i;
#endif
#ifdef SERVO6_LINE
  servos.servo6_idx = config_get_by_name("SERVO6 index", 0)->val.i;
  servos.servo6_failsafe = config_get_by_name("SERVO6 failsafe", 0)->val.i;
#endif
#ifdef SERVO7_LINE
  servos.servo7_idx = config_get_by_name("SERVO7 index", 0)->val.i;
  servos.servo7_failsafe = config_get_by_name("SERVO7 failsafe", 0)->val.i;
#endif

  // Initialize the servos which are used
  board_init_servos(
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
    (servos.servo7_idx != 255)
#endif
  );

  // Set all servos to the initial failsafe value
  board_set_servos(true,
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
    servos.servo7_failsafe
#endif
  );

  // Initialize the servo timeout timer
  chVTObjectInit(&servos.timeout_vt);
  servos.initialized = true;
}

void servos_disable(void) {
  servos.initialized = false;
  board_disable_servos();
}

/*
  handle a RAW_COMMAND request
 */
void handle_esc_rawcommand(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
  if(!servos.initialized)
    return;
  
  // Decode the commands
  uint8_t cnt = (transfer->payload_len * 8) / 14;
  int16_t commands[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH];
  uint32_t offset = 0;

  for(uint8_t i = 0; i < cnt; i++) {
    canardDecodeScalar(transfer, offset, 14, true, (void*)&commands[i]);
    offset += 14;
  }

  // Set the target commands for the servo
#ifdef SERVO1_LINE
  int16_t servo1_cmd;
  if(servos.servo1_idx < cnt) {
    servo1_cmd = 1500+(commands[servos.servo1_idx]*500/8191);
    if(servo1_cmd < 0) servo1_cmd = 0;
  } else {
    servo1_cmd = servos.servo1_failsafe;
  }
#endif
#ifdef SERVO2_LINE
  int16_t servo2_cmd;
  if(servos.servo2_idx < cnt) {
    servo2_cmd = 1500+(commands[servos.servo2_idx]*500/8191);
    if(servo2_cmd < 0) servo2_cmd = 0;
  } else {
    servo2_cmd = servos.servo2_failsafe;
  }
#endif
#ifdef SERVO3_LINE
  int16_t servo3_cmd;
  if(servos.servo3_idx < cnt) {
    servo3_cmd = 1500+(commands[servos.servo3_idx]*500/8191);
    if(servo3_cmd < 0) servo3_cmd = 0;
  } else {
    servo3_cmd = servos.servo3_failsafe;
  }
#endif
#ifdef SERVO4_LINE
  int16_t servo4_cmd;
  if(servos.servo4_idx < cnt) {
    servo4_cmd = 1500+(commands[servos.servo4_idx]*500/8191);
    if(servo4_cmd < 0) servo4_cmd = 0;
  } else {
    servo4_cmd = servos.servo4_failsafe;
  }
#endif
#ifdef SERVO5_LINE
  int16_t servo5_cmd;
  if(servos.servo5_idx < cnt) {
    servo5_cmd = 1500+(commands[servos.servo5_idx]*500/8191);
    if(servo5_cmd < 0) servo5_cmd = 0;
  } else {
    servo5_cmd = servos.servo5_failsafe;
  }
#endif
#ifdef SERVO6_LINE
  int16_t servo6_cmd;
  if(servos.servo6_idx < cnt) {
    servo6_cmd = 1500+(commands[servos.servo6_idx]*500/8191);
    if(servo6_cmd < 0) servo6_cmd = 0;
  } else {
    servo6_cmd = servos.servo6_failsafe;
  }
#endif
#ifdef SERVO7_LINE
  int16_t servo7_cmd;
  if(servos.servo7_idx < cnt) {
    servo7_cmd = 1500+(commands[servos.servo7_idx]*500/8191);
    if(servo7_cmd < 0) servo7_cmd = 0;
  } else {
    servo7_cmd = servos.servo7_failsafe;
  }
#endif

  // Commit the commands
  board_set_servos(true,
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
    servo7_cmd
#endif
  );

  // Enable timeout
  chVTSet(&servos.timeout_vt, TIME_MS2I(servos.node_timeout), servos_timeout_cb, NULL);
}