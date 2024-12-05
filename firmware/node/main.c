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
#include <stdlib.h>

#include <ch.h>
#include <hal.h>
#include "chprintf.h"
#include "config.h"
#include "uavcan.h"
#include "adcs.h"
#include "drs_parachute.h"
#include "servos.h"
#include "faulhaber_ctrl.h"
#include "esc_telem.h"
#include "tfmini.h"
#include "ie_fuelcell.h"

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

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
  adcs_init();
  drs_parachute_init();
  servos_init();
  faulhaber_ctrl_init();
  esc_telem_init();
  tfmini_init();
  ie_fuelcell_init();

  /*
   * Normal main() thread activity, spawning shells.
   */
  while (true) {
    chThdSleepMilliseconds(500);
    palToggleLine(LED1_LINE);
  }
}

//#pragma GCC pop_options