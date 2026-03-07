/**
 * @file lvgl_driver.cpp
 * @brief LVGL driver implementation for TFT_eSPI with full rendering support
 */

#include "lvgl_driver.h"

#include "os_lib.h"

#include <Arduino.h>
#include <esp_heap_caps.h>


/* Display buffer for LVGL - allocated from PSRAM */
#define DISP_BUF_SIZE (LV_HOR_RES_MAX * 40)
static lv_color_t *disp_buf1 = nullptr;
static lv_color_t *disp_buf2 = nullptr;

/* Forward declarations */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static void lvgl_input_read_cb(lv_indev_t *indev, lv_indev_data_t *data);

/* Static reference to driver for callbacks */
static lvgl_driver_t *g_lvgl_driver                              = nullptr;
static void (*g_touch_callback)(lv_indev_t *, lv_indev_data_t *) = nullptr;
static uint32_t g_last_tick                                      = 0;

/**
 * Flush callback - called by LVGL to update display
 */
void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  if (g_lvgl_driver == nullptr || g_lvgl_driver->tft == nullptr)
  {
    lv_display_flush_ready(disp);
    return;
  }

  TFT_eSPI *tft = g_lvgl_driver->tft;

  uint16_t w = (area->x2 - area->x1 + 1);
  uint16_t h = (area->y2 - area->y1 + 1);

  tft->startWrite();
  tft->setAddrWindow(area->x1, area->y1, w, h);
  tft->pushColors((uint16_t *) px_map, w * h, true);
  tft->endWrite();

  lv_display_flush_ready(disp);
}

/**
 * Input read callback - called by LVGL to read touch input
 */
void lvgl_input_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  if (g_touch_callback != nullptr)
  {
    g_touch_callback(indev, data);
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

/**
 * Initialize LVGL driver with full rendering support
 */
void bsp_lvgl_init(lvgl_driver_t *ctx, TFT_eSPI *tft)
{
  if (ctx == nullptr || tft == nullptr)
  {
    return;
  }

  g_lvgl_driver = ctx;
  ctx->tft      = tft;
  g_last_tick   = OS_GET_TICK();

  /* Allocate display buffers from PSRAM */
  size_t buf_size = DISP_BUF_SIZE * sizeof(lv_color_t);
  disp_buf1       = (lv_color_t *) heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  disp_buf2       = (lv_color_t *) heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);

  if (disp_buf1 == nullptr || disp_buf2 == nullptr)
  {
    printf("[LVGL] ERROR: Failed to allocate display buffers from PSRAM\n");
    /* Fallback to internal RAM */
    if (disp_buf1 == nullptr)
      disp_buf1 = (lv_color_t *) malloc(buf_size);
    if (disp_buf2 == nullptr)
      disp_buf2 = (lv_color_t *) malloc(buf_size);
    printf("[LVGL] Using internal RAM for display buffers\n");
  }
  else
  {
    printf("[LVGL] Display buffers allocated from PSRAM (%d bytes each)\n", buf_size);
  }

  printf("[LVGL] Calling lv_init()...\n");
  lv_init();
  printf("[LVGL] lv_init() completed\n");

  printf("[LVGL] Creating display...\n");
  ctx->display = lv_display_create(LV_HOR_RES_MAX, LV_VER_RES_MAX);
  if (ctx->display == nullptr)
  {
    printf("[LVGL] ERROR: Failed to create display\n");
    return;
  }
  printf("[LVGL] Display created\n");

  /* Enable flush callback for LVGL rendering */
  lv_display_set_flush_cb(ctx->display, lvgl_flush_cb);
  lv_display_set_buffers(ctx->display, disp_buf1, disp_buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_rotation(ctx->display, LV_DISPLAY_ROTATION_0);

  printf("[LVGL] Creating input device...\n");
  ctx->input_device = lv_indev_create();
  if (ctx->input_device == nullptr)
  {
    printf("[LVGL] ERROR: Failed to create input device\n");
    return;
  }
  printf("[LVGL] Input device created\n");

  lv_indev_set_type(ctx->input_device, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(ctx->input_device, lvgl_input_read_cb);
  lv_indev_set_user_data(ctx->input_device, ctx->user_data);
  lv_indev_set_display(ctx->input_device, ctx->display);
  printf("[LVGL] Input device configured\n");
}

/**
 * Handle LVGL timer tasks (call in main loop)
 */
void bsp_lvgl_task(lvgl_driver_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  /* Update LVGL tick */
  uint32_t now     = OS_GET_TICK();
  uint32_t elapsed = now - g_last_tick;
  g_last_tick      = now;
  lv_tick_inc(elapsed);

  /* Process LVGL tasks */
  lv_timer_handler();
}

/**
 * Set custom touch callback
 */
void bsp_lvgl_set_touch_callback(lvgl_driver_t *ctx, void (*read_cb)(lv_indev_t *, lv_indev_data_t *))
{
  if (ctx != nullptr)
  {
    g_touch_callback = read_cb;
  }
}

/**
 * Get the active display from driver
 */
lv_display_t *bsp_lvgl_get_display(lvgl_driver_t *ctx)
{
  if (ctx != nullptr)
  {
    return ctx->display;
  }
  return nullptr;
}
