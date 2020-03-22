/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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
 * Setup for the STM32F103C8T6 proto board.
 */

/*
 * Board identifier.
 */
#define STM32_MB_SWITCH
#define BOARD_NAME              "STM32F103 modbus switch"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xB

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0              0
#define GPIOA_PIN1              1
#define GPIOA_PIN2              2
#define GPIOA_PIN3              3
#define GPIOA_PIN4              4
#define GPIOA_PIN5              5
#define GPIOA_PIN6              6
#define GPIOA_PIN7              7
#define GPIOA_MB_TXEN           8
#define GPIOA_MB_TX             9
#define GPIOA_MB_RX             10
#define GPIOA_PIN11             11
#define GPIOA_PIN12             12
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14
#define GPIOA_PIN15             15


#define GPIOB_LED               0
#define GPIOB_IN2               1
#define GPIOB_OUT2              2
#define GPIOB_OUT1              3
#define GPIOB_IN1               4
#define GPIOB_PIN5              5
#define GPIOB_PIN6              6
#define GPIOB_PIN7              7
#define GPIOB_PIN8              8
#define GPIOB_PIN9              9
#define GPIOB_PIN10             10
#define GPIOB_PIN11             11
#define GPIOB_PIN12             12
#define GPIOB_PIN13             13
#define GPIOB_PIN14             14
#define GPIOB_PIN15             15

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
 * Everything input with pull-up except:
 * PA9  - Alternate output  (USART1 TX).
 * PA10 - Normal input      (USART1 RX).
 * PA11 - Normal input      (USB DM).
 * PA12 - Normal input      (USB DP).
 */
#define VAL_GPIOACRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888884B3      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOBCRL            0x88883343      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
//#define usb_lld_connect_bus(usbp) palClearPad(GPIOC, GPIOC_USB_DISC)
#define usb_lld_connect_bus(usbp) palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_INPUT);

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) { \
 palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_OUTPUT_PUSHPULL); \
 palClearPad(GPIOA, GPIOA_USB_DP); \
}

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
