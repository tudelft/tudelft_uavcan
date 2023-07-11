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
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;
    AFIO->MAPR |= AFIO_MAPR_CAN2_REMAP;
    AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
    
    //AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NOJNTRST;
    AFIO->MAPR &= ~AFIO_MAPR_SWJ_CFG;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // Enable PA15
}

#if HAL_USE_PWM
static bool servo_enabled[7] = {
  false,
  false,
  false,
  false,
  false,
  false, 
  false
};

void board_init_servos(bool servo1, bool servo2, bool servo3, bool servo4, bool servo5, bool servo6, bool servo7) {
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

  if(servo1) {
    palSetLineMode(SERVO1_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim5.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[0] = true;
  }
  if(servo2) {
    palSetLineMode(SERVO2_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim5.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[1] = true;
  }
  if(servo3) {
    palSetLineMode(SERVO3_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim1.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[2] = true;
  }
  if(servo4) {
    palSetLineMode(SERVO4_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim2.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[3] = true;
  }
  if(servo5) {
    palSetLineMode(SERVO5_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim5.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[4] = true;
  }
  if(servo6) {
    palSetLineMode(SERVO6_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim1.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[5] = true;
  }
  if(servo7) {
    palSetLineMode(SERVO7_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    pwmcfg_tim2.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
    servo_enabled[6] = true;
  }

  if(servo_enabled[0] || servo_enabled[1] || servo_enabled[4]) {
    pwmStart(&PWMD5, &pwmcfg_tim5);
  }
  if(servo_enabled[2] || servo_enabled[5]) {
    pwmStart(&PWMD1, &pwmcfg_tim1);
  }
  if(servo_enabled[3] || servo_enabled[6]) {
    pwmStart(&PWMD2, &pwmcfg_tim2);
  }
}

void board_disable_servos(void) {
  if(servo_enabled[0] || servo_enabled[1] || servo_enabled[4]) {
    pwmStop(&PWMD5);
  }
  if(servo_enabled[2] || servo_enabled[5]) {
    pwmStop(&PWMD1);
  }
  if(servo_enabled[3] || servo_enabled[6]) {
    pwmStop(&PWMD2);
  }
}

void board_set_servos(uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4, uint16_t servo5, uint16_t servo6, uint16_t servo7) {
  if(servo_enabled[0] && servo1 != 0) {
    pwmEnableChannelI(&PWMD5, 3, servo1);
  }
  if(servo_enabled[1] && servo2 != 0) {
    pwmEnableChannelI(&PWMD5, 0, servo2);
  }
  if(servo_enabled[2] && servo3 != 0) {
    pwmEnableChannelI(&PWMD1, 2, servo3);
  }
  if(servo_enabled[3] && servo4 != 0) {
    pwmEnableChannelI(&PWMD2, 3, servo4);
  }
  if(servo_enabled[4] && servo5 != 0) {
    pwmEnableChannelI(&PWMD5, 2, servo5);
  }
  if(servo_enabled[5] && servo6 != 0) {
    pwmEnableChannelI(&PWMD1, 1, servo6);
  }
  if(servo_enabled[6] && servo7 != 0) {
    pwmEnableChannelI(&PWMD2, 2, servo7);
  }
}
#endif