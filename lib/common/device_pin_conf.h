/**
 * @file       device_pin_conf.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Device pin configuration definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _DEVICE_PIN_CONF_H_
#define _DEVICE_PIN_CONF_H_
/* Includes ----------------------------------------------------------- */
/* Public defines ----------------------------------------------------- */

// IL9341 screen configuration
#define SCREEN_MISO         (13)
#define SCREEN_MOSI         (11)
#define SCREEN_SCLK         (12)
#define SCREEN_CS           (10)
#define SCREEN_DC           (9)
#define SCREEN_RST          (8)
#define SCREEN_BL           (14)
#define SCREEN_TOUCH_CS     (18)
#define SCREEN_TOUCH_IRQ    (8)

// UART configuration
#define SIM_UART_HANDLER    UART_NUM_2
#define SIM_UART_TX         (43)  // Connect to SIM RX
#define SIM_UART_RX         (44)  // Connect to SIM TX
#define SIM_UART_BAUDRATE   (115200)

#define GPS_UART_HANDLER    UART_NUM_1
#define GPS_UART_TX         (17)  // Connect to GPS RX
#define GPS_UART_RX         (16)  // Connect to GPS TX
#define GPS_UART_BAUDRATE   (9600)

// GP2Y1010AU0F Dust Sensor configuration
#define DUST_SENSOR_LED_PIN (2)
#define DUST_SENSOR_VO_PIN  (1)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif /*End file _DEVICE_PIN_CONF_H_*/

/* End of file -------------------------------------------------------- */
