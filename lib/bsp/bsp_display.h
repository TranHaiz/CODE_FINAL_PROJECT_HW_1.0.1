/**
 * @file       bsp_display.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      Board support abstraction for TFT display hardware (Singleton)
 *
 * @note       This module provides low-level hardware access only.
 *             All UI/widget logic should be at sys_ui layer.
 *             Only supports single instance.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_DISPLAY_H_
#define _BSP_DISPLAY_H_

/* Includes ----------------------------------------------------------- */
#include <Arduino.h>
#include <TFT_eSPI.h>

/* Public defines ----------------------------------------------------- */
#define BSP_DISPLAY_SCREEN_W (320)
#define BSP_DISPLAY_SCREEN_H (240)

// clang-format off
#define BSP_DISPLAY_RGB_TO_HEX(r, g, b) \
(((size_t) (r) << 16) | ((size_t) (g) << 8) | (size_t) (b))

#define BSP_DISPLAY_RGB_TO_565(r, g, b) \
  ((((uint16_t) (r) >> 3) << 11) | (((uint16_t) (g) >> 2) << 5) | ((uint16_t) (b) >> 3))

#define BSP_DISPLAY_HEX_TO_565(hex) \
  ((((hex >> 16) & 0xFF) >> 3) << 11) | ((((hex >> 8) & 0xFF) >> 2) << 5) | (((hex & 0xFF) >> 3))
// clang-format on

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize display hardware
 */
void bsp_display_init(void);

/**
 * @brief Set backlight brightness percentage (0-100)
 */
void bsp_display_set_brightness_percent(int percent);

/**
 * @brief Get current brightness percentage
 */
uint8_t bsp_display_get_brightness_percent(void);

/**
 * @brief Show splash/download screen before LVGL is ready
 */
void bsp_display_show_splash(void);

/**
 * @brief Get TFT driver pointer for LVGL library
 */
TFT_eSPI *bsp_display_get_driver(void);

#endif /* _BSP_DISPLAY_H_ */

/* End of file -------------------------------------------------------- */
