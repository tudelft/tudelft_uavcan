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

#include <stdio.h>
#include <string.h>

#include <ch.h>
#include <hal.h>
#include "config.h"
#include "uavcan.h"
#include "esc.h"

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time = 500;
    //palClearLine(LED1_LINE);
    chThdSleepMilliseconds(time);
    //palSetLine(LED1_LINE);
    chThdSleepMilliseconds(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* 
   * Initialize the uavcan driver
   */
  config_init();
  uavcanInit();
  esc_init();
  volz_servo_init();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, spawning shells.
   */
  while (true) {
    // if (SDU1.config->usbp->state == USB_ACTIVE) {
    //   thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
    //                                           "shell", NORMALPRIO + 1,
    //                                           shellThread, (void *)&shell_cfg1);
    //   chThdWait(shelltp);               /* Waiting termination.             */
    // }
    chThdSleepMilliseconds(1000);
  }
}