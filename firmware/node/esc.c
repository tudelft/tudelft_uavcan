#include "esc.h"

#include "config.h"
#include "volz_servo.h"

#include <ch.h>
#include <hal.h>

struct esc_telem_data_t esc_telem_data;
uint8_t esc_idx = 0;
static uint16_t esc_failsafe = 1000;
static uint8_t servo_idx = 1;
static uint32_t node_timeout = 100;
static uint8_t servo_type = 0;
static uint16_t pwm_cmd = 1000;
static bool req_telem = false; 
static bool esc_init_done = false;

static virtual_timer_t esc_timeout_vt;
static virtual_timer_t esc_telem_vt;
static void pwm_cb(PWMDriver *pwmp);
static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg_esc = {
  1000000,                                  /* 1MHz PWM clock frequency.      */
  2500,                                     /* PWM period frequency is 400Hz. */
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, &pwm_cb},
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

/*
 * UART telemetry driver configuration.
 */
static UARTConfig uart_telem_cfg = {
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  115200,
  USART_CR1_UE | USART_CR1_RE,
  0,
  0
};

static void pwm_cb(PWMDriver *pwmp __attribute__((unused))) {
  // Reset to original pwm command
  if(req_telem) {
    req_telem = false;
    pwmEnableChannelI(&PWMD5, 0, 30);
  } else
    pwmEnableChannelI(&PWMD5, 0, pwm_cmd);
}

static void esc_timeout_cb(void *arg __attribute__((unused))) {
  // When no commands are received timeout and set ESC to failsafe
  req_telem = false;
  pwm_cmd = esc_failsafe;
  pwmEnableChannelI(&PWMD5, 0, pwm_cmd);
}

static void esc_telem_cb(void *arg __attribute__((unused))) {
  // Request for telemetry
  req_telem = true;

  chSysLockFromISR();
  pwmEnableChannelI(&PWMD5, 0, 30);
  chVTSetI(&esc_telem_vt, TIME_MS2I(500), esc_telem_cb, NULL);
  chSysUnlockFromISR();
}

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(esc_telem_wa, 128);
static THD_FUNCTION(esc_telem_thd, arg) {

  (void)arg;
  chRegSetThreadName("esc_telem");
  while (true) {
    size_t recv_size = 10;
    uint8_t resp[10];
    // Try to receive bytes
    if(uartReceiveTimeout(&UARTD1, &recv_size, (void *)resp, TIME_MS2I(350)) == MSG_OK) {
      // Verify the CRC and size
      if(recv_size == 10 && get_crc8(resp, 9) == resp[9]) {
        palToggleLine(LED1_LINE);

        uint16_t volt_u = (resp[1] << 8) | resp[2];
        uint16_t curr_u = (resp[3] << 8) | resp[4];
        uint16_t erpm_u = (resp[7] << 8) | resp[8];

        esc_telem_data.temp = (int8_t)resp[0];
        esc_telem_data.voltage = volt_u / 100.0f;
        esc_telem_data.current = curr_u / 100.0f;
        esc_telem_data.consumption = (resp[5] << 8) | resp[6];
        esc_telem_data.erpm = erpm_u * 100;
      }
    }
  }
}

void esc_init(void) {
  esc_idx = config_get_by_name("ESC index", 0)->val.i % UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH;
  esc_failsafe = config_get_by_name("ESC failsafe", 0)->val.i;
  servo_idx = config_get_by_name("SERVO index", 0)->val.i % UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH;
  servo_type = config_get_by_name("SERVO type", 0)->val.i;
  node_timeout = config_get_by_name("NODE timeout (ms)", 0)->val.i;
  pwm_cmd = esc_failsafe;

  chVTObjectInit(&esc_timeout_vt);
  chVTObjectInit(&esc_telem_vt);

  // Start the ESC PWM driver
  pwmStart(&PWMD5, &pwmcfg_esc);
  pwmEnableChannel(&PWMD5, 0, esc_failsafe);

  // Start the SERVO PWM driver
  pwmStart(&PWMD3, &pwmcfg_servo);
  pwmEnableChannel(&PWMD3, 0, 1500);

  // Start the telemetry requests
  uartStart(&UARTD1, &uart_telem_cfg);
  pwmEnableChannelNotification(&PWMD5, 0);
  chVTSet(&esc_telem_vt, TIME_MS2I(500), esc_telem_cb, NULL);
  chThdCreateStatic(esc_telem_wa, sizeof(esc_telem_wa), NORMALPRIO, esc_telem_thd, NULL);

  esc_init_done = true;
}

void esc_disable(void) {
  pwmDisableChannel(&PWMD5, 0);
  pwmDisableChannel(&PWMD3, 0);
}

/*
  handle a RAW_COMMAND request
 */
void handle_esc_rawcommand(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer* transfer)
{
  if(!esc_init_done)
    return;
  
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
  if(!req_telem)
    pwmEnableChannel(&PWMD5, 0, pwm_cmd);

  int16_t servo_cmd = 1500+(commands[servo_idx]*500/8191);
  if(servo_cmd < 0) servo_cmd = 0;
  pwmEnableChannel(&PWMD3, 0, servo_cmd);

  //volz_servo_set(commands[servo_idx]);

  // Enable timeout
  chVTSet(&esc_timeout_vt, TIME_MS2I(node_timeout), esc_timeout_cb, NULL);
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}