/**
 * @file lvgl_driver.h
 * @brief LVGL driver for TFT_eSPI on ESP32-S3 with full rendering support
 */

#ifndef _LVGL_DRIVER_H_
#define _LVGL_DRIVER_H_

#include <TFT_eSPI.h>
#include <lvgl.h>

typedef struct
{
  TFT_eSPI     *tft;
  lv_display_t *display;
  lv_indev_t   *input_device;
  void         *user_data; /* User context for callbacks */
} lvgl_driver_t;

/* Initialize LVGL with TFT_eSPI driver */
void bsp_lvgl_init(lvgl_driver_t *ctx, TFT_eSPI *tft);

/* Handle timer-based refresh */
void bsp_lvgl_task(lvgl_driver_t *ctx);

/* Set touch input callback */
void bsp_lvgl_set_touch_callback(lvgl_driver_t *ctx, void (*read_cb)(lv_indev_t *, lv_indev_data_t *));

/* Get the active display */
lv_display_t *bsp_lvgl_get_display(lvgl_driver_t *ctx);

#endif /* _LVGL_DRIVER_H_ */
