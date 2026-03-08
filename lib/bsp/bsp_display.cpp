/**
 * @file       bsp_display.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Hardware Abstraction Only)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      Board support display - hardware driver only (TFT + backlight).
 *             All UI/widget logic moved to sys_ui layer.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"

#include <SPI.h>

/* Private defines ---------------------------------------------------- */
namespace
{
#if defined(ESP32) && defined(TFT_BL)
constexpr int BACKLIGHT_PWM_CH   = 7;
constexpr int BACKLIGHT_PWM_FREQ = 5000;
constexpr int BACKLIGHT_PWM_BITS = 8;
#endif
}  // namespace

/* Private function prototypes ---------------------------------------- */
/**
 * @brief Ensure backlight PWM channel is configured
 */
static void bsp_display_ensure_backlight_channel(bsp_display_info_t *ctx);

/* Function definitions ----------------------------------------------- */
void bsp_display_init_context(bsp_display_info_t *ctx)
{
  assert(ctx != nullptr);

  ctx->brightness_percent = 80;
  ctx->backlight_en       = false;
}

void bsp_display_init_screen(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  ctx->tft.init();
  ctx->tft.setRotation(1);
  ctx->tft.fillScreen(TFT_BLACK);
  bsp_display_set_brightness_percent(ctx, ctx->brightness_percent);
}

void bsp_display_show_dowload(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  /* Use direct TFT for splash screen (before LVGL is fully ready) */
  ctx->tft.fillScreen(TFT_BLACK);
  ctx->tft.drawRect(20, 40, BSP_DISPLAY_SCREEN_W - 40, 160, TFT_CYAN);

  ctx->tft.setTextSize(3);
  ctx->tft.setTextColor(TFT_CYAN, TFT_BLACK);
  ctx->tft.setCursor(50, 95);
  ctx->tft.print("SMART BIKE");

  ctx->tft.setTextSize(2);
  ctx->tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  ctx->tft.setCursor(105, 125);
  ctx->tft.print("RENTAL");

  ctx->tft.setTextSize(1);
  ctx->tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  ctx->tft.setCursor(125, 150);
  ctx->tft.print("Version 2.0");

  ctx->tft.drawRect(50, 175, BSP_DISPLAY_SCREEN_W - 100, 10, TFT_DARKGREY);
  for (int i = 0; i <= 100; i += 5)
  {
    int w = ((BSP_DISPLAY_SCREEN_W - 104) * i) / 100;
    ctx->tft.fillRect(52, 177, w, 6, TFT_CYAN);
    delay(20);
  }
  delay(300);
}

void bsp_display_set_brightness_percent(bsp_display_info_t *ctx, int percent)
{
  assert(ctx != nullptr);

  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;
  ctx->brightness_percent = percent;

#if defined(ESP32) && defined(TFT_BL)
  bsp_display_ensure_backlight_channel(ctx);
  size_t max_duty = (1 << BACKLIGHT_PWM_BITS) - 1;
  size_t duty     = map(ctx->brightness_percent, 0, 100, 0, max_duty);
  ledcWrite(BACKLIGHT_PWM_CH, duty);
#endif
}

uint8_t bsp_display_get_brightness_percent(const bsp_display_info_t *ctx)
{
  assert(ctx != nullptr);

  return ctx->brightness_percent;
}

TFT_eSPI *bsp_display_get_driver(bsp_display_info_t *ctx)
{
  assert(ctx != nullptr);

  return &ctx->tft;
}

/* Private definitions ----------------------------------------------- */
static void bsp_display_ensure_backlight_channel(bsp_display_info_t *ctx)
{
#if defined(ESP32) && defined(TFT_BL)
  assert(ctx != nullptr);
  if (ctx->backlight_en)
  {
    return;
  }

  ledcAttachPin(TFT_BL, BACKLIGHT_PWM_CH);
  ledcSetup(BACKLIGHT_PWM_CH, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_BITS);
  ctx->backlight_en = true;
#else
  (void) ctx;
#endif
}

/* End of file -------------------------------------------------------- */
