#include "esc.h"

#include "config.h"
#include "volz_servo.h"

#include <ch.h>
#include <hal.h>

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg = {
  1000000,                                  /* 1MHz PWM clock frequency.      */
  2500,                                     /* PWM period frequency is 400Hz. */
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  /* HW dependent part.*/
  0,
  0
};

#ifndef PWM_CMD_TO_US
#define PWM_CMD_TO_US(_t) (1000000 * _t / 400)
#endif

static uint8_t esc_idx = 0;
static uint8_t servo_idx = 1;

void esc_init(void) {
  esc_idx = config_get_by_name("ESC index", 0)->val.i % UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH;
  servo_idx = config_get_by_name("SERVO index", 0)->val.i % UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH;

  pwmStart(&PWMD5, &pwmcfg);
  pwmEnableChannel(&PWMD5, 0, 1000);
}

/*
  handle a GET_NODE_INFO request
 */
void handle_esc_rawcommand(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
  uint8_t cnt = (transfer->payload_len * 8) / 14;
  int16_t commands[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH];
  uint32_t offset = 0;

  for(uint8_t i = 0; i < cnt; i++) {
    canardDecodeScalar(transfer, offset, 14, true, (void*)&commands[i]);
    offset += 14;
  }


  int16_t esc_cmd = commands[esc_idx];
  if(esc_cmd < 0) esc_cmd = 0;

  uint16_t pwm_cmd = 1000 + (esc_cmd*1000/8191);
  pwmEnableChannel(&PWMD5, 0, pwm_cmd);

  volz_servo_set(commands[servo_idx]);
}