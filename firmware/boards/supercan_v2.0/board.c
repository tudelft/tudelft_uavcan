/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
  {VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH},
  {VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH},
  {VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH},
  {VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH},
  {VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH},
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  stm32_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Needs to be enabled before remapping

    uint32_t mapr = 0;
    mapr &= ~AFIO_MAPR_SWJ_CFG;
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // Enable PA15 and PB4

    mapr |= AFIO_MAPR_CAN_REMAP_REMAP2;
    mapr |= AFIO_MAPR_CAN2_REMAP;
    mapr |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
    mapr |= AFIO_MAPR_SPI3_REMAP;

    AFIO->MAPR = mapr;
}

#if HAL_USE_PWM
static bool servo_enabled[10] = {
  false,
  false,
  false,
  false,
  false,
  false, 
  false,
  false,
  false,
  false
};

void board_init_servos(bool servos[], uint8_t cnt) {
  PWMConfig pwmcfg_tim1 = {
    1000000,                                  /* 1MHz PWM clock frequency.      */
    2500,                                     /* PWM period frequency is 400Hz. */
    NULL,
    {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
    },
    /* HW dependent part.*/
    0,
    0
  };
  PWMConfig pwmcfg_tim2 = {
    1000000,                                  /* 1MHz PWM clock frequency.      */
    2500,                                     /* PWM period frequency is 400Hz. */
    NULL,
    {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
    },
    /* HW dependent part.*/
    0,
    0
  };
  PWMConfig pwmcfg_tim3 = {
    1000000,                                  /* 1MHz PWM clock frequency.      */
    2500,                                     /* PWM period frequency is 400Hz. */
    NULL,
    {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
    },
    /* HW dependent part.*/
    0,
    0
  };
  PWMConfig pwmcfg_tim5 = {
    1000000,                                  /* 1MHz PWM clock frequency.      */
    2500,                                     /* PWM period frequency is 400Hz. */
    NULL,
    {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
    },
    /* HW dependent part.*/
    0,
    0
  };

  if(cnt >= 1 && servos[0]) {
    palSetLineMode(SERVO1_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim5.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[0] = true;
  }
  if(cnt >= 2 && servos[1]) {
    palSetLineMode(SERVO2_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim5.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[1] = true;
  }
  if(cnt >= 3 && servos[2]) {
    palSetLineMode(SERVO3_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim1.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[2] = true;
  }
  if(cnt >= 4 && servos[3]) {
    palSetLineMode(SERVO4_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim2.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[3] = true;
  }
  if(cnt >= 5 && servos[4]) {
    palSetLineMode(SERVO5_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim5.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[4] = true;
  }
  if(cnt >= 6 && servos[5]) {
    palSetLineMode(SERVO6_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim1.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[5] = true;
  }
  if(cnt >= 7 && servos[6]) {
    palSetLineMode(SERVO7_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim2.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[6] = true;
  }
  if(cnt >= 8 && servos[7]) {
    palSetLineMode(SERVO8_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim3.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[7] = true;
  }
  if(cnt >= 9 && servos[8]) {
    palSetLineMode(SERVO9_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim3.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[8] = true;
  }
  if(cnt >= 10 && servos[9]) {
    palSetLineMode(SERVO10_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim2.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[9] = true;
  }

  if(servo_enabled[2] || servo_enabled[5]) {
    pwmStart(&PWMD1, &pwmcfg_tim1);
  }
  if(servo_enabled[3] || servo_enabled[6] || servo_enabled[9]) {
    pwmStart(&PWMD2, &pwmcfg_tim2);
  }
  if(servo_enabled[7] || servo_enabled[8]) {
    pwmStart(&PWMD3, &pwmcfg_tim3);
  }
  if(servo_enabled[0] || servo_enabled[1] || servo_enabled[4]) {
    pwmStart(&PWMD5, &pwmcfg_tim5);
  }
}

void board_disable_servos(void) {
  if(servo_enabled[2] || servo_enabled[5]) {
    pwmStop(&PWMD1);
  }
  if(servo_enabled[3] || servo_enabled[6] || servo_enabled[9]) {
    pwmStop(&PWMD2);
  }
  if(servo_enabled[7] || servo_enabled[8]) {
    pwmStop(&PWMD3);
  }
  if(servo_enabled[0] || servo_enabled[1] || servo_enabled[4]) {
    pwmStop(&PWMD5);
  }
}



void board_set_servos(bool lock, uint16_t servos[], uint8_t cnt) {
  if(lock) {
    osalSysLock();
  }

  if(servo_enabled[0] && cnt >= 1 && servos[0] != 0) {
    pwmEnableChannelI(&PWMD5, 3, servos[0]);
  }
  if(servo_enabled[1] && cnt >= 2 && servos[1] != 0) {
    pwmEnableChannelI(&PWMD5, 0, servos[1]);
  }
  if(servo_enabled[2] && cnt >= 3 && servos[2] != 0) {
    pwmEnableChannelI(&PWMD1, 2, servos[2]);
  }
  if(servo_enabled[3] && cnt >= 4 && servos[3] != 0) {
    pwmEnableChannelI(&PWMD2, 3, servos[3]);
  }
  if(servo_enabled[4] && cnt >= 5 && servos[4] != 0) {
    pwmEnableChannelI(&PWMD5, 2, servos[4]);
  }
  if(servo_enabled[5] && cnt >= 6 && servos[5] != 0) {
    pwmEnableChannelI(&PWMD1, 1, servos[5]);
  }
  if(servo_enabled[6] && cnt >= 7 && servos[6] != 0) {
    pwmEnableChannelI(&PWMD2, 2, servos[6]);
  }
  if(servo_enabled[7] && cnt >= 8 && servos[7] != 0) {
    pwmEnableChannelI(&PWMD3, 0, servos[7]);
  }
  if(servo_enabled[8] && cnt >= 9 && servos[8] != 0) {
    pwmEnableChannelI(&PWMD3, 1, servos[8]);
  }
  if(servo_enabled[9] && cnt >= 10 && servos[9] != 0) {
    pwmEnableChannelI(&PWMD2, 1, servos[9]);
  }

  if(lock) {
    osalSysUnlock();
  }
}
#endif