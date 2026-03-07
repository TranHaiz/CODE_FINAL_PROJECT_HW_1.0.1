/**
 * @file       bsp_display.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0 (LVGL Full Integration)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      Board support abstraction for the TFT UI using LVGL widgets (C style API).
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_DISPLAY_H_
#define _BSP_DISPLAY_H_
/* Includes ----------------------------------------------------------- */
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>

/* Public defines ----------------------------------------------------- */
#define BSP_DISPLAY_HEX_TO_565(hex) \
  ((((hex >> 16) & 0xFF) >> 3) << 11) | ((((hex >> 8) & 0xFF) >> 2) << 5) | (((hex & 0xFF) >> 3))

/* LVGL Colors (lv_color_hex format) */
#define BSP_DISPLAY_COLOR_BG                0x000000
#define BSP_DISPLAY_COLOR_BG_CARD           0x18E318
#define BSP_DISPLAY_COLOR_BG_DARK           0x084108
#define BSP_DISPLAY_COLOR_PRIMARY           0x00FFFF
#define BSP_DISPLAY_COLOR_ACCENT            0xFFFF00
#define BSP_DISPLAY_COLOR_SUCCESS           0x00FF00
#define BSP_DISPLAY_COLOR_WARNING           0xFFA500
#define BSP_DISPLAY_COLOR_DANGER            0xFF0000
#define BSP_DISPLAY_COLOR_TEXT              0xFFFFFF
#define BSP_DISPLAY_COLOR_TEXT_DIM          0x808080

#define BSP_DISPLAY_SCREEN_W                320
#define BSP_DISPLAY_SCREEN_H                240

#define BSP_DISPLAY_TIME_CARD_X             195
#define BSP_DISPLAY_TIME_CARD_Y             32
#define BSP_DISPLAY_TIME_CARD_W             95
#define BSP_DISPLAY_TIME_CARD_H             55

#define BSP_DISPLAY_DIST_CARD_X             195
#define BSP_DISPLAY_DIST_CARD_Y             92
#define BSP_DISPLAY_DIST_CARD_W             95
#define BSP_DISPLAY_DIST_CARD_H             42

#define BSP_DISPLAY_ENV_CARD_X              195
#define BSP_DISPLAY_ENV_CARD_Y              139
#define BSP_DISPLAY_ENV_CARD_W              95
#define BSP_DISPLAY_ENV_CARD_H              78

#define BSP_DISPLAY_CTRL_BTN_Y              32
#define BSP_DISPLAY_CTRL_BTN_H              24
#define BSP_DISPLAY_SETTINGS_BTN_X          20
#define BSP_DISPLAY_SETTINGS_BTN_W          80
#define BSP_DISPLAY_OUT_BTN_X               105
#define BSP_DISPLAY_OUT_BTN_W               80

#define BSP_DISPLAY_BACK_BTN_X              10
#define BSP_DISPLAY_BACK_BTN_Y              10
#define BSP_DISPLAY_BACK_BTN_W              60
#define BSP_DISPLAY_BACK_BTN_H              25

#define BSP_DISPLAY_EXTEND_BTN_X            180
#define BSP_DISPLAY_EXTEND_BTN_Y            180
#define BSP_DISPLAY_EXTEND_BTN_W            120
#define BSP_DISPLAY_EXTEND_BTN_H            30

#define BSP_DISPLAY_SETTINGS_BRIGHT_MINUS_X 40
#define BSP_DISPLAY_SETTINGS_BRIGHT_PLUS_X  230
#define BSP_DISPLAY_SETTINGS_BRIGHT_BTN_Y   90
#define BSP_DISPLAY_SETTINGS_BRIGHT_BTN_W   50
#define BSP_DISPLAY_SETTINGS_BRIGHT_BTN_H   30

#define BSP_DISPLAY_SETTINGS_COLOR_ROW_Y    150
#define BSP_DISPLAY_SETTINGS_COLOR_SIZE     36
#define BSP_DISPLAY_SETTINGS_COLOR_SPAN     70

#define BSP_DISPLAY_TEMP_GRAPH_X            20
#define BSP_DISPLAY_TEMP_GRAPH_Y            50
#define BSP_DISPLAY_TEMP_GRAPH_W            280
#define BSP_DISPLAY_TEMP_GRAPH_H            140

#define BSP_DISPLAY_TEMP_BTN_Y              200
#define BSP_DISPLAY_TEMP_BTN_W              60
#define BSP_DISPLAY_TEMP_BTN_H              24
#define BSP_DISPLAY_TEMP_BTN_GAP            10

/* LVGL Screen types */
typedef enum
{
  BSP_DISPLAY_SCREEN_MAIN = 0,
  BSP_DISPLAY_SCREEN_SETTINGS,
  BSP_DISPLAY_SCREEN_OUT,
  BSP_DISPLAY_SCREEN_TIME_HISTORY,
  BSP_DISPLAY_SCREEN_DISTANCE,
  BSP_DISPLAY_SCREEN_TEMPERATURE
} bsp_display_screen_type_t;

typedef struct
{
  TFT_eSPI  tft;
  lv_obj_t *active_screen;

  /* Main screen widgets */
  lv_obj_t *main_screen;
  lv_obj_t *header_panel;
  lv_obj_t *header_title;
  lv_obj_t *header_subtitle;
  lv_obj_t *status_led;
  lv_obj_t *battery_bar;
  lv_obj_t *speedometer_arc;
  lv_obj_t *speed_label;
  lv_obj_t *speed_unit_label;
  lv_obj_t *time_card;
  lv_obj_t *time_label;
  lv_obj_t *time_unit_label;
  lv_obj_t *distance_card;
  lv_obj_t *distance_label;
  lv_obj_t *distance_unit_label;
  lv_obj_t *env_card;
  lv_obj_t *temp_label;
  lv_obj_t *humidity_label;
  lv_obj_t *aqi_label;
  lv_obj_t *settings_btn;
  lv_obj_t *out_btn;

  /* Settings screen widgets */
  lv_obj_t *settings_screen;
  lv_obj_t *settings_title;
  lv_obj_t *settings_back_btn;
  lv_obj_t *brightness_slider;
  lv_obj_t *brightness_label;
  lv_obj_t *color_btns[3];

  /* Out screen widgets */
  lv_obj_t *out_screen;
  lv_obj_t *out_title;
  lv_obj_t *out_back_btn;

  /* Time history screen widgets */
  lv_obj_t *time_history_screen;
  lv_obj_t *time_history_title;
  lv_obj_t *time_back_btn;
  lv_obj_t *time_remaining_label;
  lv_obj_t *history_labels[4];
  lv_obj_t *extend_btn;

  /* Distance screen widgets */
  lv_obj_t          *distance_screen;
  lv_obj_t          *distance_title;
  lv_obj_t          *distance_back_btn;
  lv_obj_t          *total_distance_label;
  lv_obj_t          *avg_speed_label;
  lv_obj_t          *distance_chart;
  lv_chart_series_t *distance_series;

  /* Temperature screen widgets */
  lv_obj_t          *temp_screen;
  lv_obj_t          *temp_title;
  lv_obj_t          *temp_back_btn;
  lv_obj_t          *temp_chart;
  lv_chart_series_t *temp_series;
  lv_obj_t          *temp_range_label;
  lv_obj_t          *zoom_minus_btn;
  lv_obj_t          *zoom_plus_btn;
  lv_obj_t          *pan_left_btn;
  lv_obj_t          *pan_right_btn;

  /* Generic back button (used in overlays) */
  lv_obj_t *back_btn;

  /* State */
  int                       prev_speed_int;
  int                       battery_percent;
  uint32_t                  background_color;
  int                       brightness_percent;
  bool                      backlight_en;
  bsp_display_screen_type_t current_screen;
} bsp_display_info_t;

/* Public function prototypes ----------------------------------------- */
void      bsp_display_initContext(bsp_display_info_t *ctx);
void      bsp_display_initialize(bsp_display_info_t *ctx);
void      bsp_display_show_splash(bsp_display_info_t *ctx);
void      bsp_display_set_battery_percent(bsp_display_info_t *ctx, int percent);
void      bsp_display_draw_full_ui(bsp_display_info_t *ctx,
                                   int                 minutes,
                                   int                 seconds,
                                   float               distance_km,
                                   float               current_speed,
                                   float               temperature_C,
                                   float               humidity,
                                   int                 air_quality);
void      bsp_display_updateSpeed(bsp_display_info_t *ctx, int speedKph);
void      bsp_display_updateTime(bsp_display_info_t *ctx, int minutes, int seconds);
void      bsp_display_updateDistance(bsp_display_info_t *ctx, float distance_km);
void      bsp_display_updateEnvironment(bsp_display_info_t *ctx, float temperature_C, float humidity, int air_quality);
void      bsp_display_set_background_color(bsp_display_info_t *ctx, uint16_t color);
uint16_t  bsp_display_cvt_background_color(const bsp_display_info_t *ctx);
void      bsp_display_set_brightness_percent(bsp_display_info_t *ctx, int percent);
int       bsp_display_brightnessPercent(const bsp_display_info_t *ctx);
void      bsp_display_drawSettingsOverlay(bsp_display_info_t *ctx, int brightness_percent, uint16_t bgColor);
void      bsp_display_drawOutOverlay(bsp_display_info_t *ctx);
void      bsp_display_drawTimeHistoryOverlay(bsp_display_info_t *ctx,
                                             const char         *history[],
                                             int                 count,
                                             int                 remaining_minutes,
                                             int                 remaining_seconds);
void      bsp_display_drawDistanceOverlay(bsp_display_info_t *ctx,
                                          float               avgSpeed,
                                          float               totalKm,
                                          const float        *history,
                                          int                 count);
void      bsp_display_drawTemperatureOverlay(bsp_display_info_t *ctx,
                                             const float        *samples,
                                             const uint32_t     *timestamps,
                                             int                 count,
                                             int                 zoomLevel,
                                             int                 panOffset);
TFT_eSPI *bsp_display_driver(bsp_display_info_t *ctx);

#endif /*End file _BSP_DISPLAY_H_*/

/* End of file -------------------------------------------------------- */
