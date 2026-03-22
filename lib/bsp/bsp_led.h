/**
 * @file       bsp_led.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     LED control interface, WWS120 RGB LED
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_LED_H_
#define _BSP_LED_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
#define BSP_LED_TIME_PULSE_HIGH_MS       (500)
#define BSP_LED_TIME_PULSE_HIGH_SOLID_MS (100)
#define BSP_LED_TIME_PULSE_LOW_MS        (500)
#define BSP_LED_TIME_PULSE_LOW_SOLID_MS  (100)
#define BSP_LED_TIME_FLASH_SLOW_MS       (1000)
#define BSP_LED_TIME_FLASH_FAST_MS       (300)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  LED_COLOR_GREEN = 0,
  LED_COLOR_BLUE,
  LED_COLOR_YELLOW,
  LED_COLOR_CYAN,
  LED_COLOR_WHITE,
  LED_COLOR_RED,
  LED_COLOR_PINK,
  LED_COLOR_PURPLE,
  LED_COLOR_MAX
} bsp_led_color_t;

typedef enum
{
  LED_MODE_SOLID = 0,
  LED_MODE_FLASH_SLOW,
  LED_MODE_FLASH_FAST,
  LED_MODE_PULSE
} bsp_led_mode_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the LED
 *
 * @param[in] tick_ms Tick interval for LED update task (ms)
 *
 * @return none
 */
void bsp_led_init(uint32_t tick_ms);

/**
 * @brief Run the LED with specified color, mode, and brightness
 *
 * @param[in] color LED color (bsp_led_color_t)
 * @param[in] mode LED mode (bsp_led_mode_t)
 * @param[in] brightness Brightness level (0-100%)
 *
 * @return none
 */
void bsp_led_set(bsp_led_color_t color, bsp_led_mode_t mode, uint8_t brightness);

/**
 * @brief Polling task to update LED state (non-blocking)
 *
 * @return none
 */
void bsp_led_task(void);

/**
 * @brief Turn off the LED
 *
 * @return none
 */
void bsp_led_off(void);

#endif /*End file _BSP_LED_H_*/

/* End of file -------------------------------------------------------- */
