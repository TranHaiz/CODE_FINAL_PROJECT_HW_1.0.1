/**
 * @file       sys_ui.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Full UI Layer)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      System level UI controller with all LVGL widgets and screen management.
 *             Owns all UI state; bsp_display only provides hardware access.
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_UI_H_
#define _SYS_UI_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"
#include "bsp_touch.h"
#include "lvgl_driver.h"
#include "sys_ui_widget.h"

#include <Arduino.h>
#include <lvgl.h>

/* Public defines ----------------------------------------------------- */
/* Timing constants */
#define SYS_UI_SPEED_REFRESH_MS   50UL
#define SYS_UI_COUNTDOWN_MS       1000UL
#define SYS_UI_ENVIRONMENT_MS     3000UL
#define SYS_UI_MAX_RENTAL_HISTORY 4
#define SYS_UI_MAX_DISTANCE_LOG   16
#define SYS_UI_MAX_TEMP_SAMPLES   120

/* Color aliases - use widget colors */
#define SYS_UI_COLOR_BG           SYS_UI_WIDGET_COLOR_BG
#define SYS_UI_COLOR_BG_CARD      SYS_UI_WIDGET_COLOR_BG_CARD
#define SYS_UI_COLOR_PRIMARY      SYS_UI_WIDGET_COLOR_PRIMARY
#define SYS_UI_COLOR_ACCENT       SYS_UI_WIDGET_COLOR_ACCENT
#define SYS_UI_COLOR_SUCCESS      SYS_UI_WIDGET_COLOR_SUCCESS
#define SYS_UI_COLOR_WARNING      SYS_UI_WIDGET_COLOR_WARNING
#define SYS_UI_COLOR_DANGER       SYS_UI_WIDGET_COLOR_DANGER
#define SYS_UI_COLOR_TEXT         SYS_UI_WIDGET_COLOR_TEXT
#define SYS_UI_COLOR_TEXT_DIM     SYS_UI_WIDGET_COLOR_TEXT_DIM

/* UI Layout Constants */
#define SYS_UI_TIME_CARD_X        (195)
#define SYS_UI_TIME_CARD_Y        (32)
#define SYS_UI_TIME_CARD_W        (95)
#define SYS_UI_TIME_CARD_H        (55)

#define SYS_UI_DIST_CARD_X        195
#define SYS_UI_DIST_CARD_Y        92
#define SYS_UI_DIST_CARD_W        95
#define SYS_UI_DIST_CARD_H        42

#define SYS_UI_ENV_CARD_X         195
#define SYS_UI_ENV_CARD_Y         139
#define SYS_UI_ENV_CARD_W         95
#define SYS_UI_ENV_CARD_H         78

#define SYS_UI_CTRL_BTN_Y         32
#define SYS_UI_CTRL_BTN_H         24
#define SYS_UI_SETTINGS_BTN_X     20
#define SYS_UI_SETTINGS_BTN_W     80
#define SYS_UI_OUT_BTN_X          105
#define SYS_UI_OUT_BTN_W          80

#define SYS_UI_BACK_BTN_X         10
#define SYS_UI_BACK_BTN_Y         10
#define SYS_UI_BACK_BTN_W         60
#define SYS_UI_BACK_BTN_H         25

#define SYS_UI_EXTEND_BTN_X       180
#define SYS_UI_EXTEND_BTN_Y       180
#define SYS_UI_EXTEND_BTN_W       120
#define SYS_UI_EXTEND_BTN_H       30

#define SYS_UI_COLOR_ROW_Y        150
#define SYS_UI_COLOR_SIZE         36
#define SYS_UI_COLOR_SPAN         70

#define SYS_UI_TEMP_GRAPH_X       20
#define SYS_UI_TEMP_GRAPH_Y       50
#define SYS_UI_TEMP_GRAPH_W       280
#define SYS_UI_TEMP_GRAPH_H       140
#define SYS_UI_TEMP_BTN_Y         200
#define SYS_UI_TEMP_BTN_W         60
#define SYS_UI_TEMP_BTN_H         24
#define SYS_UI_TEMP_BTN_GAP       10

/* Speedometer layout */
#define SYS_UI_SPEEDO_CX          90
#define SYS_UI_SPEEDO_CY          150
#define SYS_UI_SPEEDO_OUTER_R     70
#define SYS_UI_SPEEDO_INNER_R     50

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_UI_VIEW_MAIN = 0,
  SYS_UI_VIEW_SETTINGS,
  SYS_UI_VIEW_OUT,
  SYS_UI_VIEW_TIME,
  SYS_UI_VIEW_DISTANCE,
  SYS_UI_VIEW_TEMPERATURE
} sys_ui_view_t;

/**
 * @brief UI widget collection (all LVGL objects)
 */
typedef struct
{
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
} sys_ui_widgets_t;


/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize UI system (display, LVGL, widgets)
 */
void sys_ui_init(void);

/**
 * @brief Process UI loop iteration (update telemetry, refresh display)
 */
void sys_ui_process(void);

#endif /*End file _SYS_UI_H_*/

/* End of file -------------------------------------------------------- */
