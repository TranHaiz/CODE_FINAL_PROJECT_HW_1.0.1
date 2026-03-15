/**
 * @file       bsp_touch.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      Touch helper implementation (Singleton)
 *
 * @note       Shares the TFT SPI bus via bsp_display singleton.
 *             Only supports single instance.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_touch.h"

#include "bsp_display.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Internal touch context (singleton)
 */
typedef struct
{
  TFT_eSPI *tft;
  bool      invert_x;
  bool      invert_y;
  bool      is_initialized;
} bsp_touch_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_touch_ctx_t s_touch_ctx = {
  .tft            = nullptr,
  .invert_x       = false,
  .invert_y       = true,
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

void bsp_touch_init(void)
{
  if (s_touch_ctx.is_initialized)
  {
    return;
  }

  // Get TFT driver from display singleton
  s_touch_ctx.tft = bsp_display_get_driver();

  if (s_touch_ctx.tft == nullptr)
  {
    return;
  }

#if defined(TOUCH_IRQ)
  // Configure the IRQ pin as input with internal pull-up (IRQ is active LOW)
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
#endif

  s_touch_ctx.is_initialized = true;
}

bool bsp_touch_read(bsp_touch_point_t *point)
{
  if (point == nullptr || !s_touch_ctx.is_initialized || s_touch_ctx.tft == nullptr)
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
  uint16_t raw_x = 0;
  uint16_t raw_y = 0;
  bool     hit  = s_touch_ctx.tft->getTouch(&raw_x, &raw_y);

  point->touched = hit;
  if (!hit)
  {
    point->x = 0;
    point->y = 0;
    return false;
  }

  uint16_t width  = s_touch_ctx.tft->width();
  uint16_t height = s_touch_ctx.tft->height();

  uint16_t inv_x = (width > 0) ? static_cast<uint16_t>((width - 1) - raw_x) : raw_x;
  uint16_t inv_y = (height > 0) ? static_cast<uint16_t>((height - 1) - raw_y) : raw_y;

  point->x = s_touch_ctx.invert_x ? inv_x : raw_x;
  point->y = s_touch_ctx.invert_y ? inv_y : raw_y;

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
