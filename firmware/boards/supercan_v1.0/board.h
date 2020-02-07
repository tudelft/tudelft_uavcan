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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the LeafLabs Maple Mini.
 */

/*
 * Board identifier.
 */
#define BOARD_SUPERCAN_V1
#define BOARD_NAME              "Supercan V1.0"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            12000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F105xC

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK

/*
 * IO pins assignments
 */

#define ESC_LINE          PAL_LINE(GPIOA, 0U)
#define LED1_LINE         PAL_LINE(GPIOA, 9U)
#define CAN1_STBY_LINE    PAL_LINE(GPIOB, 4U)
#define CAN2_STBY_LINE    PAL_LINE(GPIOB, 7U)
#define RS485_DE_LINE     PAL_LINE(GPIOA, 15U)
#define RS485_RE_LINE     PAL_LINE(GPIOC, 0U)

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * PA0  - B - Push Pull output 50MHz.          (PWM1)
 * PA1  - 0 - Analog input                     (ADC1)
 * PA2  - 0 - Analog input                     (ADC2)
 * PA3  - 0 - Analog input                     (ADC3)
 * PA4  - 0 - Analog input                     (ADC4)
 * PA5  - 0 - Analog input                     (v1_sense)
 * PA6  - B - Push Pull output 50MHz.          (PWM2)
 * PA7  - 8 - Digital input.                   (NC)
 * PA8  - 8 - Digital input.                   (NC)
 * PA9  - 7 - Open Drain output 50MHz.         (LED)
 * PA10 - 4 - Digital input.                   (UART1_RX)
 * PA11 - 8 - Digital input                    (NC)
 * PA12 - 8 - Digital input                    (NC)
 * PA13 - 4 - Digital input                    (JTAG_TMS)
 * PA14 - 4 - Digital input                    (JTAG_TCK)
 * PA15 - 3 - Push Pull output 50MHz.          (RS485_DE)
 */
#define VAL_GPIOACRL            0x8B00000B      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x34488478      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup:
 * PB0  - 8 - Digital input                    (NC)
 * PB1  - 8 - Digital input                    (NC)
 * PB2  - 8 - Digital input                    (NC)
 * PB3  - 8 - Digital input                    (NC)
 * PB4  - F - Alternate Open Drain output 50MHz.  (CAN1_STBY)
 * PB5  - 4 - Digital input                    (CAN2_RX)
 * PB6  - B - Alternate Open Drain output 50MHz (CAN2_TX)
 * PB7  - 7 - Open Drain output 50MHz.          (CAN2_STBY)
 * PB8  - 4 - Digital input.                   (CAN1_RX)
 * PB9  - B - Alternate Open Drain output 50MHz.(CAN1_TX)
 * PB10 - 8 -                                  (NC)
 * PB11 - 8 -                                  (NC)
 * PB12 - 8 -                                  (NC)
 * PB13 - 8 -                                  (NC)
 * PB14 - 8 -                                  (NC)
 * PB15 - 8 -                                  (NC)
 */
#define VAL_GPIOBCRL            0x7B4B8888      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x888888B4      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup:
 * PC0  - 8 - Analog input                     (NC)
 * PC1  - 8 - Analog input                     (NC)
 * PC2  - 8 - Open Drain output 50MHz          (NC)
 * PC3  - 8 - Analog input                     (NC)
 * PC4  - 8 - Analog input                     (NC)
 * PC5  - 7 - Open Drain output 50MHz          (v2_drive)
 * PC6  - 8 - Alternate Push Pull output 50MHz (NC)
 * PC7  - 8 - Alternate Push Pull output 50MHz (NC)
 * PC8  - 8 - Alternate Push Pull output 50MHz (NC)
 * PC9  - 8 - Alternate Push Pull output 50MHz (NC)
 * PC10 - B - Alternate Push Pull output 50MHz (UART3_TX)
 * PC11 - 4 - Digital input                    (UART3_RX)
 * PC12 - 8 - Alternate Push Pull output 50MHz (NC)
 * PC13 - 8 - Push Pull output 50MHz.          (NC)
 * PC14 - 8 - Digital input                    (NC)
 * PC15 - 8 - Open Drain output 50MHz          (NC)
 */
#define VAL_GPIOCCRL            0x88788888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88884B88      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup:
 * PD0  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_IN).
 * PD1  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_OUT).
 * PD2  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD3  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD4  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD5  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD6  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD7  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD8  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD9  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD10 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD11 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD12 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD13 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD14 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD15 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 */
#define VAL_GPIODCRL            0x88888888      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * ALL  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).)
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */