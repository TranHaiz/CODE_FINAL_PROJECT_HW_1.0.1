/**
 * @file       device_config.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Device pin, handler, i2c address,.. configuration definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _DEVICE_CONFIG_H_
#define _DEVICE_CONFIG_H_
/* Includes ----------------------------------------------------------- */
/* Public defines ----------------------------------------------------- */

// IL9341 screen configuration
#define SCREEN_MISO                 (3)
#define SCREEN_MOSI                 (10)
#define SCREEN_SCLK                 (9)
#define SCREEN_CS                   (13)
#define SCREEN_DC                   (11)
#define SCREEN_RST                  (12)
#define SCREEN_BL                   (46)
#define SCREEN_TOUCH_CS             (8)
#define SCREEN_TOUCH_IRQ            (18)

// UART configuration
#define SIM_UART_HANDLER            UART_NUM_2
#define SIM_UART_TX                 (42)  // Connect to SIM RX
#define SIM_UART_RX                 (2)  // Connect to SIM TX
#define SIM_UART_BAUDRATE           (115200)

#define GPS_UART_HANDLER            UART_NUM_1
#define GPS_UART_TX                 (43)  // Connect to GPS RX
#define GPS_UART_RX                 (44)  // Connect to GPS TX
#define GPS_UART_BAUDRATE           (9600)

// GP2Y1010AU0F Dust Sensor configuration
#define DUST_SENSOR_LED_PIN         (7)
#define DUST_SENSOR_AOOUT_PIN       (6)

// LSM6DS3 Accelerometer I2C configuration
#define ACC_I2C_SDA_PIN             (4)
#define ACC_I2C_SCL_PIN             (5)
#define ACC_INT_PIN                 (6)
#define ACC_I2C_ADDR                (0x6B)
#define ACC_I2C_ADDR_ALT            (0x6A)

// HMC5883L Compass I2C configuration
#define COMPASS_I2C_SDA_PIN         (4)
#define COMPASS_I2C_SCL_PIN         (5)
#define COMPASS_I2C_ADDR            (0x1E)
#define COMPASS_I2C_CLOCK           (100000)

// SHT31 Temperature and Humidity Sensor I2C configuration
#define TEMP_HUM_I2C_SDA_PIN        (4)
#define TEMP_HUM_I2C_SCL_PIN        (5)
#define TEMP_HUM_I2C_ADDR           (0x44)
#define TEMP_HUM_I2C_CLOCK          (100000)

// DS1307 RTC I2C configuration
#define RTC_I2C_SDA_PIN             (40)
#define RTC_I2C_SCL_PIN             (39)
#define RTC_I2C_ADDR                (0x68)

// SD Card SPI configuration
#define BSP_SDCARD_DEFAULT_CS_PIN   (21)
#define BSP_SDCARD_DEFAULT_SCK_PIN  (47)
#define BSP_SDCARD_DEFAULT_MOSI_PIN (48)
#define BSP_SDCARD_DEFAULT_MISO_PIN (45)
#define SDCARD_MIN_SPI_FREQ         (1000000)   // 1MHz min
#define SDCARD_NORMAL_SPI_FREQ      (4000000)   // 4MHz normal
#define SDCARD_MAX_SPI_FREQ         (24000000)  // 24MHz max

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif /*End file _DEVICE_CONFIG_H_*/

/* End of file -------------------------------------------------------- */
