/**
 * @file       bsp_display.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      Board support display - hardware driver only (Singleton)
 *
 * @note       This module provides low-level hardware access only.
 *             All UI/widget logic should be at sys_ui layer.
 *             Only supports single instance.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"

#include <SPI.h>

/* Private defines ---------------------------------------------------- */
#if defined(ESP32) && defined(TFT_BL)
static constexpr int BACKLIGHT_PWM_CH   = 7;
static constexpr int BACKLIGHT_PWM_FREQ = 5000;
static constexpr int BACKLIGHT_PWM_BITS = 8;
#endif

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Internal display context (singleton)
 */
typedef struct
{
  TFT_eSPI tft;
  uint8_t  brightness_percent;
  bool     backlight_en;
  bool     is_initialized;
} bsp_display_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_display_ctx_t s_display_ctx = {
  .tft                = TFT_eSPI(),
  .brightness_percent = 80,
  .backlight_en       = false,
  .is_initialized     = false,
};

/* Private function prototypes ---------------------------------------- */
static void bsp_display_ensure_backlight_channel(void);

/* Function definitions ----------------------------------------------- */

void bsp_display_init(void)
{
  if (s_display_ctx.is_initialized)
  {
    return;
  }

  s_display_ctx.tft.init();
  s_display_ctx.tft.setRotation(1);
  s_display_ctx.tft.fillScreen(TFT_BLACK);

  bsp_display_set_brightness_percent(s_display_ctx.brightness_percent);
  s_display_ctx.is_initialized = true;
}

void bsp_display_set_brightness_percent(int percent)
{
  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;

  s_display_ctx.brightness_percent = percent;

#if defined(ESP32) && defined(TFT_BL)
  bsp_display_ensure_backlight_channel();
  size_t max_duty = (1 << BACKLIGHT_PWM_BITS) - 1;
  size_t duty     = map(s_display_ctx.brightness_percent, 0, 100, 0, max_duty);
  ledcWrite(BACKLIGHT_PWM_CH, duty);
#endif
}

uint8_t bsp_display_get_brightness_percent(void)
{
  return s_display_ctx.brightness_percent;
}

void bsp_display_show_splash(void)
{
  /* Use direct TFT for splash screen (before LVGL is fully ready) */
  s_display_ctx.tft.fillScreen(TFT_BLACK);
  s_display_ctx.tft.drawRect(20, 40, BSP_DISPLAY_SCREEN_W - 40, 160, TFT_CYAN);

  s_display_ctx.tft.setTextSize(3);
  s_display_ctx.tft.setTextColor(TFT_CYAN, TFT_BLACK);
  s_display_ctx.tft.setCursor(50, 95);
  s_display_ctx.tft.print("SMART BIKE");

  s_display_ctx.tft.setTextSize(2);
  s_display_ctx.tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  s_display_ctx.tft.setCursor(105, 125);
  s_display_ctx.tft.print("RENTAL");

  s_display_ctx.tft.setTextSize(1);
  s_display_ctx.tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  s_display_ctx.tft.setCursor(125, 150);
  s_display_ctx.tft.print("Version 2.0");

  s_display_ctx.tft.drawRect(50, 175, BSP_DISPLAY_SCREEN_W - 100, 10, TFT_DARKGREY);
  for (int i = 0; i <= 100; i += 5)
  {
    int w = ((BSP_DISPLAY_SCREEN_W - 104) * i) / 100;
    s_display_ctx.tft.fillRect(52, 177, w, 6, TFT_CYAN);
    delay(20);
  }
  delay(300);
}

TFT_eSPI *bsp_display_get_driver(void)
{
  return &s_display_ctx.tft;
}

/* Private definitions ----------------------------------------------- */

static void bsp_display_ensure_backlight_channel(void)
{
#if defined(ESP32) && defined(TFT_BL)
  if (s_display_ctx.backlight_en)
  {
    return;
  }

  ledcAttachPin(TFT_BL, BACKLIGHT_PWM_CH);
  ledcSetup(BACKLIGHT_PWM_CH, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_BITS);
  s_display_ctx.backlight_en = true;
#endif
}

/* End of file -------------------------------------------------------- */
