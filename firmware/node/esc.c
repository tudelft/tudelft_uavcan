#include "esc.h"

#include "config.h"
#include "volz_servo.h"

#include <ch.h>
#include <hal.h>

static uint8_t esc_idx = 0;
static uint16_t esc_failsafe = 1000;
static uint8_t servo_idx = 1;
static uint32_t node_timeout = 100;
static uint8_t servo_type = 0;
static uint16_t pwm_cmd = 1000;
static bool req_telem = false; 

static virtual_timer_t esc_timeout_vt;
static void pwm_cb(PWMDriver *pwmp);

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg_esc = {
  1000000,                                  /* 1MHz PWM clock frequency.      */
  2500,                                     /* PWM period frequency is 400Hz. */
  &pwm_cb,
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

static PWMConfig pwmcfg_servo = {
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

static void pwm_cb(PWMDriver *pwmp __attribute__((unused))) {
  //if(req_telem)
  //  pwmEnableChannelI(&PWMD5, 0, pwm_cmd);
}

static void esc_timeout_cb(void *arg __attribute__((unused))) {
  pwmEnableChannelI(&PWMD5, 0, esc_failsafe);
}

void esc_init(void) {
  esc_idx = config_get_by_name("ESC index", 0)->val.i % UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH;
  esc_failsafe = config_get_by_name("ESC failsafe", 0)->val.i;
  servo_idx = config_get_by_name("SERVO index", 0)->val.i % UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH;
  servo_type = config_get_by_name("SERVO type", 0)->val.i;
  node_timeout = config_get_by_name("NODE timeout (ms)", 0)->val.i;
  pwm_cmd = esc_failsafe;

  chVTObjectInit(&esc_timeout_vt);

  // Start the ESC PWM driver
  pwmStart(&PWMD5, &pwmcfg_esc);
  pwmEnableChannel(&PWMD5, 0, esc_failsafe);

  // Start the SERVO PWM driver
  pwmStart(&PWMD3, &pwmcfg_servo);
  pwmEnableChannel(&PWMD3, 0, 1500);
}

/*
  handle a RAW_COMMAND request
 */
void handle_esc_rawcommand(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
  uint8_t cnt = (transfer->payload_len * 8) / 14;
  int16_t commands[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH];
  uint32_t offset = 0;

  if(esc_idx >= cnt || servo_idx >= cnt)
    return;

  for(uint8_t i = 0; i < cnt; i++) {
    canardDecodeScalar(transfer, offset, 14, true, (void*)&commands[i]);
    offset += 14;
  }


  int16_t esc_cmd = commands[esc_idx];
  if(esc_cmd < 0) esc_cmd = 0;

  pwm_cmd = 1000 + (esc_cmd*1000/8191);
  pwmEnableChannel(&PWMD5, 0, pwm_cmd);

  int16_t servo_cmd = 1500+(commands[servo_idx]*500/8191);
  if(servo_cmd < 0) servo_cmd = 0;
  pwmEnableChannel(&PWMD3, 0, servo_cmd);

  //volz_servo_set(commands[servo_idx]);

  // Enable timeout
  chVTSet(&esc_timeout_vt, TIME_MS2I(node_timeout), esc_timeout_cb, NULL);
}