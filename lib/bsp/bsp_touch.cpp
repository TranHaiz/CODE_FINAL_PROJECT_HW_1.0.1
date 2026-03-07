/**
 * @file       bsp_touch.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    0.1.0
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      Touch helper implementation for a TFT_eSPI shared bus.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_touch.h"

/* Private defines ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_touch_init(bsp_touch_t *ctx, TFT_eSPI *driver)
{
  if (ctx == nullptr)
  {
    return;
  }
  ctx->tft      = driver;
  ctx->invert_x = false;
  ctx->invert_y = true;

#if defined(TOUCH_IRQ)
  // Configure the IRQ pin as input with internal pull-up (IRQ is active LOW)
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
#endif
}

bool bsp_touch_read(bsp_touch_t *ctx, bsp_touch_point_t *point)
{
  if (ctx == nullptr || point == nullptr || ctx->tft == nullptr)
  {
    return false;
  }

#if !defined(TOUCH_CS)
  // Touch hardware not enabled inside TFT_eSPI configuration.
  point->touched = false;
  point->x       = 0;
  point->y       = 0;
  return false;
#else
  uint16_t rawX = 0;
  uint16_t rawY = 0;
  bool     hit  = ctx->tft->getTouch(&rawX, &rawY);

  point->touched = hit;
  if (!hit)
  {
    point->x = 0;
    point->y = 0;
    return false;
  }

  uint16_t width  = ctx->tft->width();
  uint16_t height = ctx->tft->height();

  uint16_t invX = (width > 0) ? static_cast<uint16_t>((width - 1) - rawX) : rawX;
  uint16_t invY = (height > 0) ? static_cast<uint16_t>((height - 1) - rawY) : rawY;

  point->x = ctx->invert_x ? invX : rawX;
  point->y = ctx->invert_y ? invY : rawY;

  return true;
#endif
}

bool bsp_touch_is_pressed(void)
{
#if defined(TOUCH_IRQ)
  // IRQ pin is active LOW when touch is detected
  return (digitalRead(TOUCH_IRQ) == LOW);
#else
  // No IRQ configured; fallback to polling
  return false;
#endif
}

/* End of file -------------------------------------------------------- */
