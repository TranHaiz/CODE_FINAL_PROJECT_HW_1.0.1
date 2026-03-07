/**
 * @file lv_conf.h
 * LVGL Configuration for ESP32-S3 with TFT_eSPI (LVGL v9)
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/* Display configuration */
#define LV_HOR_RES_MAX                     320
#define LV_VER_RES_MAX                     240
#define LV_COLOR_DEPTH                     16
#define LV_COLOR_16_SWAP                   0

/* Rendering configuration */
#define LV_DISP_DEF_REFR_PERIOD            33
#define LV_INDEV_DEF_READ_PERIOD           30

/* Feature enablement */
#define LV_USE_LOG                         1
#define LV_LOG_LEVEL                       LV_LOG_LEVEL_WARN

/* Font configuration - enable multiple sizes for UI */
#define LV_FONT_STD                        1
#define LV_FONT_MONTSERRAT_8               1
#define LV_FONT_MONTSERRAT_10              1
#define LV_FONT_MONTSERRAT_12              1
#define LV_FONT_MONTSERRAT_14              1
#define LV_FONT_MONTSERRAT_16              1
#define LV_FONT_MONTSERRAT_18              1
#define LV_FONT_MONTSERRAT_20              1
#define LV_FONT_MONTSERRAT_22              1
#define LV_FONT_MONTSERRAT_24              1
#define LV_FONT_MONTSERRAT_28              1
#define LV_FONT_MONTSERRAT_32              1
#define LV_FONT_MONTSERRAT_36              1
#define LV_FONT_DEFAULT                    &lv_font_montserrat_14

/* Widget configuration */
#define LV_USE_ARC                         1
#define LV_USE_BAR                         1
#define LV_USE_BTN                         1
#define LV_USE_BTNMATRIX                   1
#define LV_USE_CANVAS                      0
#define LV_USE_CHECKBOX                    1
#define LV_USE_CHART                       1
#define LV_USE_DROPDOWN                    1
#define LV_USE_IMG                         1
#define LV_USE_LABEL                       1
#define LV_USE_LINE                        1
#define LV_USE_ROLLER                      1
#define LV_USE_SLIDER                      1
#define LV_USE_SWITCH                      1
#define LV_USE_TEXTAREA                    1
#define LV_USE_TABLE                       1
#define LV_USE_TABVIEW                     1
#define LV_USE_TILEVIEW                    1
#define LV_USE_WIN                         1
#define LV_USE_SPAN                        1
#define LV_USE_SPINBOX                     1
#define LV_USE_SPINNER                     1
#define LV_USE_METER                       1
#define LV_USE_MSGBOX                      1
#define LV_USE_LED                         1
#define LV_USE_SCALE                       1

/* Theme configuration */
#define LV_USE_THEME_DEFAULT               1

/* Debug & Logging */
#define LV_USE_ASSERT_NULL                 1
#define LV_USE_ASSERT_MALLOC               1

/* Memory configuration - Use PSRAM via custom allocator */
#define LV_MEM_CUSTOM                      1
#define LV_MEM_CUSTOM_INCLUDE              <esp_heap_caps.h>
#define LV_MEM_CUSTOM_ALLOC(size)          heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
#define LV_MEM_CUSTOM_FREE(p)              heap_caps_free(p)
#define LV_MEM_CUSTOM_REALLOC(p, new_size) heap_caps_realloc(p, new_size, MALLOC_CAP_SPIRAM)

#endif /* LV_CONF_H */
