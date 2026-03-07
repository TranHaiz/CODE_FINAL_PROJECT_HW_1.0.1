/**
 * @file       bsp_display.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Hardware Abstraction Only)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      Board support abstraction for TFT display hardware.
 *
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
(((uint32_t) (r) << 16) | ((uint32_t) (g) << 8) | (uint32_t) (b))

#define BSP_DISPLAY_RGB_TO_565(r, g, b) \
  ((((uint16_t) (r) >> 3) << 11) | (((uint16_t) (g) >> 2) << 5) | ((uint16_t) (b) >> 3))

#define BSP_DISPLAY_HEX_TO_565(hex) \
  ((((hex >> 16) & 0xFF) >> 3) << 11) | ((((hex >> 8) & 0xFF) >> 2) << 5) | (((hex & 0xFF) >> 3))
// clang-format on

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Hardware display context
 */
typedef struct
{
  TFT_eSPI tft;
  uint8_t  brightness_percent;
  bool     backlight_en;
} bsp_display_info_t;

/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize display variables with defaults
 */
void bsp_display_init_context(bsp_display_info_t *ctx);

/**
 * @brief Initialize TFT hardware (SPI, rotation, backlight)
 */
void bsp_display_init_screen(bsp_display_info_t *ctx);

/**
 * @brief Set backlight brightness percentage (0-100)
 */
void bsp_display_set_brightness_percent(bsp_display_info_t *ctx, int percent);

/**
 * @brief Get current brightness percentage
 */
uint8_t bsp_display_get_brightness_percent(const bsp_display_info_t *ctx);

/**
 * @brief Show download screen using direct TFT drawing.
 *        Use it first before LVGL is fully initialized.
 */
void bsp_display_show_dowload(bsp_display_info_t *ctx);

/**
 * @brief Get TFT driver pointer for LVGL library
 */
TFT_eSPI *bsp_display_get_driver(bsp_display_info_t *ctx);

#endif /*End file _BSP_DISPLAY_H_*/

/* End of file -------------------------------------------------------- */
