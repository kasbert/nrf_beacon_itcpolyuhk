/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BEACON_H
#define BEACON_H

// LEDs definitions for beacon
#define LEDS_NUMBER    3

#define xLED_START      17
#define LED_RGB_RED    17
#define LED_RGB_GREEN  18
#define LED_RGB_BLUE   19
#define xLED_STOP       19

#define LED_RGB_RED_MASK    (1<<LED_RGB_RED)
#define LED_RGB_GREEN_MASK  (1<<LED_RGB_GREEN)
#define LED_RGB_BLUE_MASK   (1<<LED_RGB_BLUE)

#define LEDS_LIST { LED_RGB_RED, LED_RGB_GREEN, LED_RGB_BLUE}
// defining RGB led as 3 single LEDs
#define BSP_LED_0 LED_RGB_GREEN
#define BSP_LED_1 LED_RGB_RED
#define BSP_LED_2 LED_RGB_BLUE

#define BSP_LED_0_MASK    (1<<BSP_LED_0)
#define BSP_LED_1_MASK    (1<<BSP_LED_1)
#define BSP_LED_2_MASK    (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)
//#define LEDS_MASK      (BSP_LED_1_MASK)
//defines which LEDs are lit when signal is low
#define LEDS_INV_MASK  LEDS_MASK
//#define LEDS_INV_MASK  0x00000000

#define xBUTTON_START   28
#define BUTTON_0       28
#define BUTTON_1       30
#define xBUTTON_STOP    30
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BSP_BUTTON_0   BUTTON_0
#define BSP_BUTTON_1   BUTTON_1

#define BSP_BUTTON_0_MASK (1<<BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BUTTON_1)
#define BUTTONS_LIST { BUTTON_0, BUTTON_1 }

#define BUTTONS_NUMBER 2
#define BUTTONS_MASK   (BSP_BUTTON_0_MASK | BSP_BUTTON_1_MASK)
//0x00000000

// UART connection with J-Link
#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

// Low frequency clock source to be used by the SoftDevice
#define xNRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
				 .rc_ctiv       = 0,                                \
				 .rc_temp_ctiv  = 0,                                \
				 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}


#endif
