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

#include <ch.h>
#include <hal.h>
#include "uavcan.h"
#include "firmware_update.h"

#define APP_START_ADDRESS     0x08008000

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
  chSysLock();    

  asm volatile(
    "mov sp, %0	\n"
    "msr msp, %0	\n"
    "bx	%1	\n"
    : : "r"(stacktop), "r"(entrypoint) :);
}

void jump_to_app(void)
{
  const uint32_t *app_base = (const uint32_t *)(APP_START_ADDRESS);

  /*
    * We hold back the programming of the lead words until the upload
    * is marked complete by the host. So if they are not 0xffffffff,
    * we should try booting it.
    */
  if (app_base[0] == 0xffffffff) {
    return;
  }

  /*
    * The second word of the app is the entrypoint; it must point within the
    * flash area (or we have a bad flash).
    */
  if (app_base[1] < APP_START_ADDRESS) {
    return;
  }

  /*if (app_base[1] >= (APP_START_ADDRESS + board_info.fw_size)) {
      return;
  }*/
  
  // disable all interrupt sources
  port_disable();

  /* switch exception handlers to the application */
  SCB->VTOR = APP_START_ADDRESS;

  /* extract the stack and entrypoint from the app vector table and go */
  do_jump(app_base[0], app_base[1]);
}

/*
 * Application entry point.
 */
int main(void) {
  bool goto_app = true;
  if (*((uint32_t *)0x20004FF0) == 0xDEADBEEF) {
    *((uint32_t *)0x20004FF0) = 0x0;
    goto_app = false;
  }


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
  uavcanInit();

  /*
   * Normal main() thread activity, wait to boot normal app
   */
  while (true) {
    chThdSleepMilliseconds(150);
    if(!firmware_update.in_progress && goto_app)
      jump_to_app();
  }
}