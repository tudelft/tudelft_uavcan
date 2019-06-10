#include "esc.h"

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

void esc_init(void) {
  pwmStart(&PWMD5, &pwmcfg);
  pwmEnableChannel(&PWMD5, 0, 1000);
}

/*
  handle a GET_NODE_INFO request
 */
void handle_esc_rawcommand(struct uavcan_iface_t *iface, CanardRxTransfer* transfer)
{
  int16_t cmd0 = 0;
  canardDecodeScalar(transfer, 0, 14, true, (void*)&cmd0);
  if(cmd0 < 0) cmd0 = 0;

  uint16_t pwm_cmd = 1000 + (cmd0*1000/8191);
  pwmEnableChannel(&PWMD5, 0, pwm_cmd);

  int16_t cmd1 = 0;
  canardDecodeScalar(transfer, 14, 14, true, (void*)&cmd1);
  volz_servo_set(cmd1);
}