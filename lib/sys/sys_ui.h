/**
 * @file       sys_ui.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    0.1.0
 * @date       2026-02-03
 * @author     Hai Tran
 *
 * @brief      System level UI controller that drives the BSP display APIs.
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_UI_H_
#define _SYS_UI_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"
#include "bsp_touch.h"
#include "lvgl_driver.h"

#include <Arduino.h>
#include <lvgl.h>

/* Public defines ----------------------------------------------------- */
#define SYS_UI_SPEED_REFRESH_MS   50UL
#define SYS_UI_COUNTDOWN_MS       1000UL
#define SYS_UI_ENVIRONMENT_MS     3000UL
#define SYS_UI_MAX_RENTAL_HISTORY 4
#define SYS_UI_MAX_DISTANCE_LOG   16
#define SYS_UI_MAX_TEMP_SAMPLES   120

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

typedef struct
{
  // Hardware interfaces
  bsp_display_info_t display;
  bsp_touch_t        touch;
  lvgl_driver_t      lvgl;
  uint32_t           last_speed_update;
  uint32_t           last_second_tick;
  uint32_t           last_time_update_screen;

  // UI context/state
  // Speed and distance tracking
  uint32_t frame_counter;
  float    current_speed;
  float    target_speed;
  float    distance_km;
  // Countdown timer
  int remaining_minutes;
  int remaining_seconds;
  // Environment telemetry
  float temperature_C;
  float humidity;
  int   air_quality;
  int   battery_percent;
  int   brightness_percent;
  // Setting/theme
  uint16_t theme_background_color;
  // State tracking
  uint32_t      session_start_ms;
  sys_ui_view_t view;
  uint16_t      last_touch_x;
  uint16_t      last_touch_y;
  bool          pending_main_redraw;
  char          rentalHistory[SYS_UI_MAX_RENTAL_HISTORY][32];
  int           rental_history_count;
  float         distanceHistory[SYS_UI_MAX_DISTANCE_LOG];
  int           distance_history_count;
  float         temperatureHistory[SYS_UI_MAX_TEMP_SAMPLES];
  uint32_t      temperatureTimestamps[SYS_UI_MAX_TEMP_SAMPLES];
  int           temperature_sample_count;
  int           temperature_zoom;
  int           temperature_pan;
} sys_ui_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern sys_ui_t g_ui;

/* Public function prototypes ----------------------------------------- */

void sys_ui_begin(sys_ui_t *ctx);
void sys_ui_run(sys_ui_t *ctx);

#endif /*End file _SYS_UI_H_*/

/* End of file -------------------------------------------------------- */
