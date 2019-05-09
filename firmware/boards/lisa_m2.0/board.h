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
#define BOARD_LISA_M2
#define BOARD_NAME              "Lisa M2.0"

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

#define LED1_PORT   GPIOA
#define LED1_PIN    8

#define LED2_PORT   GPIOB
#define LED2_PIN    4

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
 * PA0  - 4 - Digital input                    (PPM_IN)
 *      - B - Alternate Push Pull output 50MHz (SERVO5)
 * PA1  - B - Alternate Push Pull output 50MHz (SERVO6)
 * PA2  - B - Alternate Push Pull output 50MHz (UART2_TX)
 * PA3  - 4 - Digital input                    (UART2_RX)
 * PA4  - B - Alternate Push Pull output 50MHz (EXTSPI_SS)
 * PA5  - B - Alternate Push Pull output 50MHz (EXTSPI_SCK)
 * PA6  - 4 - Digital input.                   (EXTSPI_MISO)
 * PA7  - B - Alternate Push Pull output 50MHz (EXTSPI_MOSI)
 * PA8  - 7 - Open Drain output 50MHz          (LED1)
 * PA9  - 4 - Digital input.                   (USB_VBUS)
 * PA10 - 4 - Digital input.                   (UART1_RX)/(PPM_IN TIM2_CH2)
 * PA11 - 4 - Digital input                    (USB_DM)
 * PA12 - 4 - Digital input                    (USB_DP)
 * PA13 - 4 - Digital input                    (JTAG_TMS)
 * PA14 - 4 - Digital input                    (JTAG_TCK)
 * PA15 - 4 - Digital input                    (JTAG_TDI)
 */
#define VAL_GPIOACRL            0xB4BB4BBB      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x44444447      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup:
 * PB0  - 4 - Digital input                    (BARO_DRDY)
 * PB1  - 4 - Digital input                    (EXTSPI_DRDY)
 * PB2  - 4 - Digital input                    (IMU_ACC_DRDY)
 * PB3  - 4 - Digital input                    (JTAG_TDO)
 * PB4  - 7 - Open Drain output 50MHz          (LED2)
 * PB5  - 4 - Digital input                    (IMU_MAG_DRDY)
 * PB6  - B - Alternate Push Pull output 50MHz (SERVO7)
 *      - 7 - Open Drain output 50MHz.         (I2C1_SCL)
 * PB7  - B - Alternate Push Pull output 50MHz (SERVO8)
 *      - 7 - Open Drain output 50MHz.         (I2C1_SDA)
 * PB8  - 4 - Digital input.                   (CAN_RX)
 * PB9  - B - Alternate Open Drain output 50MHz.(CAN_TX)
 * PB10 - E - Alternate Open Drain output 2MHz.(I2C2_SCL)
 * PB11 - E - Alternate Open Drain output 2MHz.(I2C2_SDA)
 * PB12 - 3 - Push Pull output 50MHz.          (IMU_ACC_CS)
 * PB13 - B - Alternate Push Pull output 50MHz (IMU_SPI_SCK)
 * PB14 - 4 - Digital input                    (IMU_SPI_MISO)
 * PB15 - B - Alternate Push Pull output 50MHz (IMU_SPI_MOSI)
 */
#define VAL_GPIOBCRL            0xBB474444      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0xB4B3EEF4      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup:
 * PC0  - 0 - Analog input                     (ADC2)
 * PC1  - 0 - Analog input                     (ADC3)
 * PC2  - 7 - Open Drain output 50MHz           (LED3)
 * PC3  - 0 - Analog input                     (ADC1)
 * PC4  - 0 - Analog input                     (VBAT_MEAS)
 * PC5  - 7 - Open Drain output 50MHz           (LED4)
 * PC6  - B - Alternate Push Pull output 50MHz (SERVO1)
 * PC7  - B - Alternate Push Pull output 50MHz (SERVO2)
 * PC8  - B - Alternate Push Pull output 50MHz (SERVO3)
 * PC9  - B - Alternate Push Pull output 50MHz (SERVO4)
 * PC10 - B - Alternate Push Pull output 50MHz (UART3_TX)
 * PC11 - 4 - Digital input                    (UART3_RX)
 * PC12 - B - Alternate Push Pull output 50MHz (PC12-UART5_TX)
 * PC13 - 3 - Push Pull output 50MHz.          (IMU_GYRO_SS)
 * PC14 - 4 - Digital input                    (IMU_GYRO_DRDY)
 * PC15 - 7 - Open Drain output 50MHz          (LED5)
 */
#define VAL_GPIOCCRL            0xBB700700      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x743B4BBB      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup:
 * PD0  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_IN).
 * PD1  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_OUT).
 * PD2  - 4 - Digital input (UART5_RX).
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
#define VAL_GPIODCRL            0x88888488      /*  PD7...PD0 */
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