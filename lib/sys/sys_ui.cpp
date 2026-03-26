/**
 * @file       sys_ui.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Full UI Layer)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      UI system implementation
 */

/* Includes ----------------------------------------------------------- */
#include "sys_ui.h"

#include "log_service.h"
#include "os_lib.h"
#include "sys_input.h"

#include <math.h>
#include <string.h>

#ifndef PI
#define PI (3.14159265358979323846)
#endif

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_ui, LOG_LEVEL_DBG);
#define SYS_UI_CLAMP(val, minv, maxv) ((val) < (minv) ? (minv) : ((val) > (maxv) ? (maxv) : (val)))

// Timing and data constants
#define SYS_UI_COUNTDOWN_MS           (1000)
#define SYS_UI_ENVIRONMENT_MS         (3000)
#define SYS_UI_MAX_RENTAL_HISTORY     (4)
#define SYS_UI_MAX_DISTANCE_LOG       (16)
#define SYS_UI_MAX_TEMP_SAMPLES       (120)

/* UI Layout Constants */
#define SYS_UI_TIME_CARD_X            (195)
#define SYS_UI_TIME_CARD_Y            (32)
#define SYS_UI_TIME_CARD_W            (95)
#define SYS_UI_TIME_CARD_H            (55)

#define SYS_UI_DIST_CARD_X            (195)
#define SYS_UI_DIST_CARD_Y            (92)
#define SYS_UI_DIST_CARD_W            (95)
#define SYS_UI_DIST_CARD_H            (42)

#define SYS_UI_ENV_CARD_X             (195)
#define SYS_UI_ENV_CARD_Y             (139)
#define SYS_UI_ENV_CARD_W             (95)
#define SYS_UI_ENV_CARD_H             (78)

#define SYS_UI_CTRL_BTN_Y             (32)
#define SYS_UI_CTRL_BTN_H             (24)
#define SYS_UI_SETTINGS_BTN_X         (20)
#define SYS_UI_SETTINGS_BTN_W         (80)
#define SYS_UI_OUT_BTN_X              (105)
#define SYS_UI_OUT_BTN_W              (80)

#define SYS_UI_BACK_BTN_X             (10)
#define SYS_UI_BACK_BTN_Y             (10)
#define SYS_UI_BACK_BTN_W             (60)
#define SYS_UI_BACK_BTN_H             (25)

#define SYS_UI_EXTEND_BTN_X           (180)
#define SYS_UI_EXTEND_BTN_Y           (180)
#define SYS_UI_EXTEND_BTN_W           (120)
#define SYS_UI_EXTEND_BTN_H           (30)

#define SYS_UI_COLOR_ROW_Y            (150)
#define SYS_UI_COLOR_SIZE             (36)
#define SYS_UI_COLOR_SPAN             (70)

#define SYS_UI_TEMP_GRAPH_X           (20)
#define SYS_UI_TEMP_GRAPH_Y           (50)
#define SYS_UI_TEMP_GRAPH_W           (280)
#define SYS_UI_TEMP_GRAPH_H           (140)
#define SYS_UI_TEMP_BTN_Y             (200)
#define SYS_UI_TEMP_BTN_W             (60)
#define SYS_UI_TEMP_BTN_H             (24)
#define SYS_UI_TEMP_BTN_GAP           (10)

/* Compass layout */
#define SYS_UI_COMPASS_CX             (70)
#define SYS_UI_COMPASS_CY             (45)
#define SYS_UI_COMPASS_R              (26)

/* Speedometer layout */
#define SYS_UI_SPEEDO_CX              (90)
#define SYS_UI_SPEEDO_CY              (150)
#define SYS_UI_SPEEDO_OUTER_R         (70)
#define SYS_UI_SPEEDO_INNER_R         (50)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  lv_obj_t *active_screen;

  /* Main screen widgets */
  lv_obj_t *main_screen;
  lv_obj_t *speedometer_arc;
  lv_obj_t *speed_label;
  lv_obj_t *speed_unit_label;
  lv_obj_t *compass_arc;
  lv_obj_t *compass_needle;
  lv_obj_t *compass_deg_label;
  lv_obj_t *compass_dir_label;
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
typedef struct
{
  /* Hardware interfaces */
  lvgl_driver_t lvgl;

  /* UI widgets */
  sys_ui_widgets_t widgets;

  /* Timing */
  size_t last_speed_update;
  size_t last_second_tick;
  size_t last_time_update_screen;

  /* Speed and distance tracking */
  size_t            frame_counter;
  float             current_speed;
  float             target_speed;
  float             distance_km;
  int               prev_speed_int;
  sys_fusion_data_t fusion;

  /* Countdown timer */
  int remaining_minutes;
  int remaining_seconds;

  /* Environment telemetry */
  float temperature_C;
  float humidity;
  int   air_quality;
  int   battery_percent;
  int   brightness_percent;
  float compass_heading_deg;

  /* Setting/theme */
  size_t background_color;

  /* State tracking */
  size_t        session_start_ms;
  sys_ui_view_t view;
  uint16_t      last_touch_x;
  uint16_t      last_touch_y;
  bool          pending_main_redraw;

  /* History data */
  char   rentalHistory[SYS_UI_MAX_RENTAL_HISTORY][32];
  int    rental_history_count;
  float  distanceHistory[SYS_UI_MAX_DISTANCE_LOG];
  int    distance_history_count;
  float  temperatureHistory[SYS_UI_MAX_TEMP_SAMPLES];
  size_t temperatureTimestamps[SYS_UI_MAX_TEMP_SAMPLES];
  int    temperature_sample_count;
  int    temperature_zoom;
  int    temperature_pan;
} sys_ui_context_t;

/* Public variables --------------------------------------------------- */
sys_ui_data_status_t g_sys_ui_data_status = { 0 };

/* Private variables -------------------------------------------------- */
sys_ui_context_t ui_ctx;

/* Private function prototypes ---------------------------------------- */
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);
static void sys_ui_settings_btn_cb(lv_event_t *e);
static void sys_ui_out_btn_cb(lv_event_t *e);
static void sys_ui_back_btn_cb(lv_event_t *e);
static void sys_ui_extend_btn_cb(lv_event_t *e);
static void sys_ui_brightness_slider_cb(lv_event_t *e);
static void sys_ui_color_btn_cb(lv_event_t *e);
static void sys_ui_zoom_minus_btn_cb(lv_event_t *e);
static void sys_ui_zoom_plus_btn_cb(lv_event_t *e);
static void sys_ui_pan_left_btn_cb(lv_event_t *e);
static void sys_ui_pan_right_btn_cb(lv_event_t *e);
static void sys_ui_time_card_cb(lv_event_t *e);
static void sys_ui_distance_card_cb(lv_event_t *e);
static void sys_ui_env_card_cb(lv_event_t *e);

static void sys_ui_init_data(void);
static void sys_ui_reset_full_widget(sys_ui_widgets_t *w);
static void sys_ui_update_speed_n_distance(void);
static void sys_ui_update_countdown(void);
static void sys_ui_update_env_data(void);
static void sys_ui_change_screen(sys_ui_view_t view);
static void sys_ui_return_main_screen(void);
static void sys_ui_log_distance_sample(float value);
static void sys_ui_logTemperatureSample(float value, size_t timestamp);
static void sys_ui_refreshTemperatureOverlay(void);
static void sys_ui_register_lvgl_callbacks(void);
static void sys_ui_show_screen(lv_obj_t *screen);
static void sys_ui_main_screen(void);
static void sys_ui_createSettingsScreen(void);
static void sys_ui_createOutScreen(void);
static void sys_ui_createTimeHistoryScreen(void);
static void sys_ui_createDistanceScreen(void);
static void sys_ui_createTemperatureScreen(void);
static void sys_ui_init_full_widget(void);
static void sys_ui_update_speed(int speedKph);
static void sys_ui_update_time(int minutes, int seconds);
static void sys_ui_update_distance(float distance_km);
static void sys_ui_update_env(float temperature_C, float humidity, int air_quality);
static void sys_ui_update_compass(void);
static void sys_ui_drawSettingsOverlay(void);
static void sys_ui_drawOutOverlay(void);
static void sys_ui_drawTimeHistoryOverlay(void);
static void sys_ui_drawDistanceOverlay(void);
static void sys_ui_drawTemperatureOverlay(void);
static void sys_ui_init_history_mock(void);

static float  s_distance_history_mock[100];
static int    s_distance_history_count = 0;
static float  s_temperature_history_mock[100];
static size_t s_temperature_time_mock[100];
static int    s_temperature_history_count = 0;

/* Function definitions ----------------------------------------------- */
void sys_ui_init(void)
{
  memset(&ui_ctx, 0, sizeof(ui_ctx));

  ui_ctx.prev_speed_int      = -1;
  ui_ctx.battery_percent     = 85;
  ui_ctx.brightness_percent  = 80;
  ui_ctx.background_color    = SYS_UI_COLOR_BG;
  ui_ctx.session_start_ms    = OS_GET_TICK();
  ui_ctx.view                = SYS_UI_VIEW_MAIN;
  ui_ctx.pending_main_redraw = false;
  ui_ctx.temperature_zoom    = 1;

  // Initialize bsp_display and reset all widget pointers to null
  sys_ui_reset_full_widget(&ui_ctx.widgets);
  bsp_display_init();

  // Initialize LVGL driver and set touch callback
  ui_ctx.lvgl.user_data = &ui_ctx;
  lvgl_driver_init(&ui_ctx.lvgl, bsp_display_get_driver());
  lvgl_driver_set_touch_callback(&ui_ctx.lvgl, sys_ui_lvgl_touch_read_cb);

  // Init UI data
  sys_ui_init_data();
  sys_ui_init_history_mock();

  // Apply initial brightness
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);

  // Create main screen and widgets
  sys_ui_init_full_widget();

  // Register LVGL event callbacks for interactive widgets
  sys_ui_register_lvgl_callbacks();

  size_t now                     = OS_GET_TICK();
  ui_ctx.session_start_ms        = now;
  ui_ctx.last_speed_update       = now;
  ui_ctx.last_second_tick        = now;
  ui_ctx.last_time_update_screen = now;

  /* Initialize touch */
  bsp_touch_init();

  LOG_DBG("[SYS_UI] Init complete\n");
}

void sys_ui_process(void)
{
  size_t now = OS_GET_TICK();

  switch (ui_ctx.view)
  {
  case SYS_UI_VIEW_MAIN:
  {
    if (now - ui_ctx.last_second_tick >= SYS_UI_COUNTDOWN_MS)
    {
      ui_ctx.last_second_tick = now;
      sys_ui_update_countdown();
      sys_ui_update_time(ui_ctx.remaining_minutes, ui_ctx.remaining_seconds);
    }

    if (g_sys_ui_data_status.is_fusion_data_ready_for_ui)
    {
      sys_input_get_fusion_data(&ui_ctx.fusion);
      sys_ui_update_speed_n_distance();
      sys_ui_update_compass();
      ui_ctx.frame_counter++;

      if ((ui_ctx.frame_counter % 60U) == 0U)
      {
        sys_ui_log_distance_sample(ui_ctx.distance_km);
      }

      g_sys_ui_data_status.is_fusion_data_ready_for_ui = false;
    }

    if (g_sys_ui_data_status.is_env_data_ready_for_ui)
    {
      sys_input_data_t env = { 0 };
      if (sys_input_get_env_data(&env) == STATUS_OK)
      {
        ui_ctx.temperature_C = env.temp_hum.temperature;
        ui_ctx.humidity      = env.temp_hum.humidity;
      }
      sys_ui_update_env(ui_ctx.temperature_C, ui_ctx.humidity, ui_ctx.air_quality);
      g_sys_ui_data_status.is_env_data_ready_for_ui = false;
    }
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    if (now - ui_ctx.last_second_tick >= SYS_UI_COUNTDOWN_MS)
    {
      ui_ctx.last_second_tick = now;
      sys_ui_update_countdown();
      sys_ui_drawTimeHistoryOverlay();
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    sys_ui_drawDistanceOverlay();
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_drawTemperatureOverlay();
    break;
  }
  case SYS_UI_VIEW_SETTINGS:
  {
    // Settings screen is mostly static except for brightness slider, so we only redraw when needed
  }
  case SYS_UI_VIEW_OUT:
  {
    // Out screen is static, no dynamic updates needed
    break;
  }
  default: break;
  }

  /* Handle pending main screen redraw */
  if (ui_ctx.view == SYS_UI_VIEW_MAIN && ui_ctx.pending_main_redraw)
  {
    sys_ui_init_full_widget();
    sys_ui_register_lvgl_callbacks();
    ui_ctx.pending_main_redraw = false;
  }

  /* Handle LVGL tasks */
  lvgl_driver_task(&ui_ctx.lvgl);
}

/* Private definitions ----------------------------------------------- */
static void sys_ui_reset_full_widget(sys_ui_widgets_t *w)
{
  w->active_screen        = nullptr;
  w->main_screen          = nullptr;
  w->speedometer_arc      = nullptr;
  w->speed_label          = nullptr;
  w->speed_unit_label     = nullptr;
  w->compass_arc          = nullptr;
  w->compass_needle       = nullptr;
  w->compass_deg_label    = nullptr;
  w->compass_dir_label    = nullptr;
  w->time_card            = nullptr;
  w->time_label           = nullptr;
  w->time_unit_label      = nullptr;
  w->distance_card        = nullptr;
  w->distance_label       = nullptr;
  w->distance_unit_label  = nullptr;
  w->env_card             = nullptr;
  w->temp_label           = nullptr;
  w->humidity_label       = nullptr;
  w->aqi_label            = nullptr;
  w->settings_btn         = nullptr;
  w->out_btn              = nullptr;
  w->settings_screen      = nullptr;
  w->settings_title       = nullptr;
  w->settings_back_btn    = nullptr;
  w->brightness_slider    = nullptr;
  w->brightness_label     = nullptr;
  w->out_screen           = nullptr;
  w->out_title            = nullptr;
  w->out_back_btn         = nullptr;
  w->time_history_screen  = nullptr;
  w->time_history_title   = nullptr;
  w->time_back_btn        = nullptr;
  w->time_remaining_label = nullptr;
  w->extend_btn           = nullptr;
  w->distance_screen      = nullptr;
  w->distance_title       = nullptr;
  w->distance_back_btn    = nullptr;
  w->total_distance_label = nullptr;
  w->avg_speed_label      = nullptr;
  w->distance_chart       = nullptr;
  w->distance_series      = nullptr;
  w->temp_screen          = nullptr;
  w->temp_title           = nullptr;
  w->temp_back_btn        = nullptr;
  w->temp_chart           = nullptr;
  w->temp_series          = nullptr;
  w->temp_range_label     = nullptr;
  w->zoom_minus_btn       = nullptr;
  w->zoom_plus_btn        = nullptr;
  w->pan_left_btn         = nullptr;
  w->pan_right_btn        = nullptr;
  for (int i = 0; i < 3; ++i) w->color_btns[i] = nullptr;
  for (int i = 0; i < 4; ++i) w->history_labels[i] = nullptr;
}

static void sys_ui_init_data(void)
{
  ui_ctx.remaining_minutes = 45;
  ui_ctx.remaining_seconds = 0;
  ui_ctx.current_speed     = 0.0f;
  ui_ctx.target_speed      = 12.0f;
  ui_ctx.distance_km       = 0.0f;

  ui_ctx.temperature_C   = 28.0f + static_cast<float>(random(0, 50)) / 10.0f;
  ui_ctx.humidity        = 60.0f + static_cast<float>(random(0, 300)) / 10.0f;
  ui_ctx.air_quality     = 70 + random(0, 30);
  ui_ctx.battery_percent = 80 + random(0, 20);

  const char *seedHistory[SYS_UI_MAX_RENTAL_HISTORY] = { "2026-03-03 09:05 - B-1024", "2026-03-02 17:40 - B-2201",
                                                         "2026-03-01 08:15 - B-3108", "2026-02-29 19:50 - B-1876" };
  ui_ctx.rental_history_count                        = SYS_UI_MAX_RENTAL_HISTORY;
  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    strncpy(ui_ctx.rentalHistory[i], seedHistory[i], sizeof(ui_ctx.rentalHistory[i]) - 1);
    ui_ctx.rentalHistory[i][sizeof(ui_ctx.rentalHistory[i]) - 1] = '\0';
  }
}

static void sys_ui_show_screen(lv_obj_t *screen)
{
  if (screen == nullptr)
    return;

  ui_ctx.widgets.active_screen = screen;
  lv_screen_load(screen);
}

static void sys_ui_main_screen(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  w->main_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  /* Speedometer Arc */
  w->speedometer_arc = sys_ui_widget_create_arc(w->main_screen, SYS_UI_SPEEDO_CX, SYS_UI_SPEEDO_CY,
                                                SYS_UI_SPEEDO_OUTER_R, 135, 405, 0x333333, SYS_UI_COLOR_SUCCESS);

  w->speed_label = sys_ui_widget_create_label(w->main_screen, SYS_UI_SPEEDO_CX - 20, SYS_UI_SPEEDO_CY - 10, "0",
                                              SYS_UI_COLOR_TEXT, &lv_font_montserrat_28);
  lv_obj_set_style_text_align(w->speed_label, LV_TEXT_ALIGN_CENTER, 0);

  w->speed_unit_label = sys_ui_widget_create_label(w->main_screen, SYS_UI_SPEEDO_CX - 15, SYS_UI_SPEEDO_CY + 20, "km/h",
                                                   SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);

  /* Compass */
  w->compass_arc = sys_ui_widget_create_arc(w->main_screen, SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY, SYS_UI_COMPASS_R, 0,
                                            360, 0x222222, SYS_UI_COLOR_PRIMARY);

  static lv_point_precise_t needle_points[2] = { { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY },
                                                 { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY - SYS_UI_COMPASS_R + 4 } };
  w->compass_needle                          = lv_line_create(w->main_screen);
  lv_line_set_points(w->compass_needle, needle_points, 2);
  lv_obj_set_style_line_width(w->compass_needle, 2, 0);
  lv_obj_set_style_line_color(w->compass_needle, lv_color_hex(SYS_UI_COLOR_ACCENT), 0);

  sys_ui_widget_create_label(w->main_screen, SYS_UI_COMPASS_CX - 4, SYS_UI_COMPASS_CY - SYS_UI_COMPASS_R - 10, "N",
                             SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  sys_ui_widget_create_label(w->main_screen, SYS_UI_COMPASS_CX + SYS_UI_COMPASS_R + 2, SYS_UI_COMPASS_CY - 5, "E",
                             SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  sys_ui_widget_create_label(w->main_screen, SYS_UI_COMPASS_CX - 4, SYS_UI_COMPASS_CY + SYS_UI_COMPASS_R + 2, "S",
                             SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  sys_ui_widget_create_label(w->main_screen, SYS_UI_COMPASS_CX - SYS_UI_COMPASS_R - 10, SYS_UI_COMPASS_CY - 5, "W",
                             SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  w->compass_deg_label = sys_ui_widget_create_label(w->main_screen, SYS_UI_COMPASS_CX - 16, SYS_UI_COMPASS_CY - 6, "0°",
                                                    SYS_UI_COLOR_TEXT, &lv_font_montserrat_10);
  w->compass_dir_label = sys_ui_widget_create_label(w->main_screen, SYS_UI_COMPASS_CX - 10, SYS_UI_COMPASS_CY + 6, "N",
                                                    SYS_UI_COLOR_ACCENT, &lv_font_montserrat_10);

  /* Time Card */
  w->time_card = sys_ui_widget_create_card(w->main_screen, SYS_UI_TIME_CARD_X, SYS_UI_TIME_CARD_Y, SYS_UI_TIME_CARD_W,
                                           SYS_UI_TIME_CARD_H, SYS_UI_COLOR_PRIMARY);

  sys_ui_widget_create_label(w->time_card, 10, 2, "REMAIN", SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);

  w->time_label =
    sys_ui_widget_create_label(w->time_card, 5, 18, "00:00", SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_18);

  w->time_unit_label =
    sys_ui_widget_create_label(w->time_card, 25, 38, "mins", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  /* Distance Card */
  w->distance_card = sys_ui_widget_create_card(w->main_screen, SYS_UI_DIST_CARD_X, SYS_UI_DIST_CARD_Y,
                                               SYS_UI_DIST_CARD_W, SYS_UI_DIST_CARD_H, SYS_UI_COLOR_ACCENT);

  sys_ui_widget_create_label(w->distance_card, 5, 2, "DISTANCE", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  w->distance_label =
    sys_ui_widget_create_label(w->distance_card, 5, 16, "0.00", SYS_UI_COLOR_ACCENT, &lv_font_montserrat_16);

  w->distance_unit_label =
    sys_ui_widget_create_label(w->distance_card, 62, 20, "km", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  /* Environment Card */
  w->env_card = sys_ui_widget_create_card(w->main_screen, SYS_UI_ENV_CARD_X, SYS_UI_ENV_CARD_Y, SYS_UI_ENV_CARD_W,
                                          SYS_UI_ENV_CARD_H, SYS_UI_COLOR_TEXT_DIM);

  sys_ui_widget_create_label(w->env_card, 8, 0, "ENV STATUS", SYS_UI_COLOR_TEXT, &lv_font_montserrat_10);

  w->temp_label = sys_ui_widget_create_label(w->env_card, 5, 16, "0.0°C", SYS_UI_COLOR_WARNING, &lv_font_montserrat_12);

  w->humidity_label =
    sys_ui_widget_create_label(w->env_card, 5, 32, "0%", SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_12);

  w->aqi_label =
    sys_ui_widget_create_label(w->env_card, 5, 48, "AQI: --", SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_10);

  /* Control Buttons */
  w->settings_btn =
    sys_ui_widget_create_button(w->main_screen, SYS_UI_SETTINGS_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_SETTINGS_BTN_W,
                                SYS_UI_CTRL_BTN_H, "SETTINGS", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);

  w->out_btn = sys_ui_widget_create_button(w->main_screen, SYS_UI_OUT_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_OUT_BTN_W,
                                           SYS_UI_CTRL_BTN_H, "OUT", SYS_UI_COLOR_WARNING, SYS_UI_COLOR_BG);
}

static void sys_ui_createSettingsScreen(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  w->settings_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  w->settings_back_btn =
    sys_ui_widget_create_button(w->settings_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->settings_title =
    sys_ui_widget_create_label(w->settings_screen, 90, 15, "SETTINGS", SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  sys_ui_widget_create_label(w->settings_screen, 40, 55, "Brightness", SYS_UI_COLOR_TEXT, nullptr);

  w->brightness_slider = sys_ui_widget_create_slider(w->settings_screen, 40, 80, 200, 20, 5, 100,
                                                     ui_ctx.brightness_percent, 0x333333, SYS_UI_COLOR_PRIMARY);

  w->brightness_label = sys_ui_widget_create_label(w->settings_screen, 250, 80, "", SYS_UI_COLOR_TEXT, nullptr);
  sys_ui_widget_set_label_text_format(w->brightness_label, "%d%%", ui_ctx.brightness_percent);

  sys_ui_widget_create_label(w->settings_screen, 40, 120, "Background", SYS_UI_COLOR_TEXT, nullptr);

  size_t colors[3] = { 0x000000, 0xFFFFFF, 0x7B7B7B };
  for (int i = 0; i < 3; ++i)
  {
    w->color_btns[i] = lv_btn_create(w->settings_screen);
    lv_obj_set_pos(w->color_btns[i], 40 + i * SYS_UI_COLOR_SPAN, SYS_UI_COLOR_ROW_Y);
    lv_obj_set_size(w->color_btns[i], SYS_UI_COLOR_SIZE, SYS_UI_COLOR_SIZE);
    lv_obj_set_style_bg_color(w->color_btns[i], lv_color_hex(colors[i]), 0);
    lv_obj_set_style_radius(w->color_btns[i], 4, 0);
  }
}

static void sys_ui_createOutScreen(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  w->out_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  w->out_back_btn = sys_ui_widget_create_button(w->out_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->out_title =
    sys_ui_widget_create_label(w->out_screen, 80, 80, "OUT MODE", SYS_UI_COLOR_WARNING, &lv_font_montserrat_24);

  sys_ui_widget_create_label(w->out_screen, 80, 130, "Tap BACK to resume.", SYS_UI_COLOR_TEXT, nullptr);
}

static void sys_ui_createTimeHistoryScreen(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  w->time_history_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  w->time_back_btn =
    sys_ui_widget_create_button(w->time_history_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->time_history_title = sys_ui_widget_create_label(w->time_history_screen, 60, 15, "RENTAL HISTORY",
                                                     SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  for (int i = 0; i < 4; ++i)
  {
    w->history_labels[i] = sys_ui_widget_create_label(w->time_history_screen, 40, 55 + i * 20, "", SYS_UI_COLOR_TEXT,
                                                      &lv_font_montserrat_12);
  }

  w->time_remaining_label =
    sys_ui_widget_create_label(w->time_history_screen, 40, 140, "Remaining: 00:00", SYS_UI_COLOR_TEXT, nullptr);

  w->extend_btn =
    sys_ui_widget_create_button(w->time_history_screen, SYS_UI_EXTEND_BTN_X, SYS_UI_EXTEND_BTN_Y, SYS_UI_EXTEND_BTN_W,
                                SYS_UI_EXTEND_BTN_H, "+30 min", SYS_UI_COLOR_SUCCESS, SYS_UI_COLOR_BG);
}

static void sys_ui_createDistanceScreen(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  w->distance_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  w->distance_back_btn =
    sys_ui_widget_create_button(w->distance_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->distance_title =
    sys_ui_widget_create_label(w->distance_screen, 70, 15, "DISTANCE STATS", SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  w->total_distance_label =
    sys_ui_widget_create_label(w->distance_screen, 30, 50, "Total: 0.00 km", SYS_UI_COLOR_TEXT, nullptr);

  w->avg_speed_label =
    sys_ui_widget_create_label(w->distance_screen, 30, 70, "Avg speed: 0.0 km/h", SYS_UI_COLOR_TEXT, nullptr);

  w->distance_chart  = sys_ui_widget_create_chart(w->distance_screen, 30, 95, 260, 120, 16, 0x111111);
  w->distance_series = sys_ui_widget_addChart_series(w->distance_chart, SYS_UI_COLOR_ACCENT);
}

static void sys_ui_createTemperatureScreen(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  w->temp_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  w->temp_back_btn =
    sys_ui_widget_create_button(w->temp_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->temp_title =
    sys_ui_widget_create_label(w->temp_screen, 90, 15, "TEMPERATURE", SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  w->temp_chart  = sys_ui_widget_create_chart(w->temp_screen, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_GRAPH_Y,
                                              SYS_UI_TEMP_GRAPH_W, SYS_UI_TEMP_GRAPH_H, 60, 0x111111);
  w->temp_series = sys_ui_widget_addChart_series(w->temp_chart, SYS_UI_COLOR_ACCENT);

  w->temp_range_label = sys_ui_widget_create_label(w->temp_screen, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_GRAPH_Y - 15,
                                                   "Collecting...", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  w->zoom_minus_btn =
    sys_ui_widget_create_button(w->temp_screen, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_BTN_Y, SYS_UI_TEMP_BTN_W,
                                SYS_UI_TEMP_BTN_H, "Zoom-", 0x333333, SYS_UI_COLOR_TEXT);

  w->zoom_plus_btn = sys_ui_widget_create_button(
    w->temp_screen, SYS_UI_TEMP_GRAPH_X + SYS_UI_TEMP_BTN_W + SYS_UI_TEMP_BTN_GAP, SYS_UI_TEMP_BTN_Y, SYS_UI_TEMP_BTN_W,
    SYS_UI_TEMP_BTN_H, "Zoom+", SYS_UI_COLOR_SUCCESS, SYS_UI_COLOR_BG);

  w->pan_left_btn = sys_ui_widget_create_button(
    w->temp_screen, SYS_UI_TEMP_GRAPH_X + 2 * (SYS_UI_TEMP_BTN_W + SYS_UI_TEMP_BTN_GAP), SYS_UI_TEMP_BTN_Y,
    SYS_UI_TEMP_BTN_W, SYS_UI_TEMP_BTN_H, "<", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);

  w->pan_right_btn = sys_ui_widget_create_button(
    w->temp_screen, SYS_UI_TEMP_GRAPH_X + 3 * (SYS_UI_TEMP_BTN_W + SYS_UI_TEMP_BTN_GAP), SYS_UI_TEMP_BTN_Y,
    SYS_UI_TEMP_BTN_W, SYS_UI_TEMP_BTN_H, ">", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);
}

static void sys_ui_init_full_widget(void)
{
  if (ui_ctx.widgets.main_screen == nullptr)
  {
    sys_ui_main_screen();
  }

  sys_ui_show_screen(ui_ctx.widgets.main_screen);
  sys_ui_update_compass();
  sys_ui_update_time(ui_ctx.remaining_minutes, ui_ctx.remaining_seconds);
  sys_ui_update_distance(ui_ctx.distance_km);
  sys_ui_update_env(ui_ctx.temperature_C, ui_ctx.humidity, ui_ctx.air_quality);
  sys_ui_update_speed(static_cast<int>(ui_ctx.current_speed));
}

static void sys_ui_update_speed(int speedKph)
{
  speedKph = SYS_UI_CLAMP(speedKph, 0, 40);

  if (ui_ctx.prev_speed_int == speedKph)
    return;
  ui_ctx.prev_speed_int = speedKph;

  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->speedometer_arc != nullptr)
  {
    int arcValue = (speedKph * 100) / 40;
    sys_ui_widget_set_arc_value(w->speedometer_arc, arcValue);

    size_t arcColor = SYS_UI_COLOR_DANGER;
    if (speedKph <= 20)
      arcColor = SYS_UI_COLOR_SUCCESS;
    else if (speedKph <= 30)
      arcColor = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_arc_color(w->speedometer_arc, arcColor);
  }

  if (w->speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->speed_label, "%d", speedKph);
  }
}

static void sys_ui_update_time(int minutes, int seconds)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;
  if (w->time_label == nullptr)
    return;

  sys_ui_widget_set_label_text_format(w->time_label, "%02d:%02d", minutes, seconds);

  size_t timeColor = SYS_UI_COLOR_SUCCESS;
  if (minutes < 5)
    timeColor = SYS_UI_COLOR_DANGER;
  else if (minutes < 10)
    timeColor = SYS_UI_COLOR_WARNING;
  sys_ui_widget_set_label_color(w->time_label, timeColor);
}

static void sys_ui_update_distance(float distance_km)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;
  if (w->distance_label == nullptr)
    return;

  sys_ui_widget_set_label_text_format(w->distance_label, "%.2f", distance_km);
}

static void sys_ui_update_env(float temperature_C, float humidity, int air_quality)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->temp_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->temp_label, "%.1f°C", temperature_C);
  }

  if (w->humidity_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->humidity_label, "%.0f%%", humidity);
  }

  if (w->aqi_label != nullptr)
  {
    const char *status = (air_quality >= 80) ? "Good" : ((air_quality >= 50) ? "Fair" : "Poor");
    sys_ui_widget_set_label_text_format(w->aqi_label, "AQI:%d %s", air_quality, status);

    size_t aqiColor = SYS_UI_COLOR_DANGER;
    if (air_quality >= 80)
      aqiColor = SYS_UI_COLOR_SUCCESS;
    else if (air_quality >= 50)
      aqiColor = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_label_color(w->aqi_label, aqiColor);
  }
}

static void sys_ui_update_speed_n_distance(void)
{
  ui_ctx.current_speed       = ui_ctx.fusion.velocity_kmh;
  ui_ctx.distance_km         = ui_ctx.fusion.distance_m / 1000.0f;
  ui_ctx.compass_heading_deg = ui_ctx.fusion.heading_deg;
  sys_ui_update_speed((int) ui_ctx.current_speed);
  sys_ui_update_distance(ui_ctx.distance_km);
}

static void sys_ui_update_countdown(void)
{
  if (ui_ctx.remaining_seconds > 0)
  {
    --ui_ctx.remaining_seconds;
  }
  else if (ui_ctx.remaining_minutes > 0)
  {
    --ui_ctx.remaining_minutes;
    ui_ctx.remaining_seconds = 59;
  }
}

static void sys_ui_update_env_data(void)
{
  ui_ctx.temperature_C = SYS_UI_CLAMP(ui_ctx.temperature_C + static_cast<float>(random(-10, 11)) / 10.0f, 20.0f, 40.0f);
  ui_ctx.humidity      = SYS_UI_CLAMP(ui_ctx.humidity + static_cast<float>(random(-20, 21)) / 10.0f, 40.0f, 95.0f);
  ui_ctx.air_quality   = SYS_UI_CLAMP(ui_ctx.air_quality + random(-5, 6), 30, 100);

  sys_ui_logTemperatureSample(ui_ctx.temperature_C, OS_GET_TICK());
}

static void sys_ui_update_compass(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;
  if (w->compass_deg_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->compass_deg_label, "%.0f°", ui_ctx.compass_heading_deg);
  }

  if (w->compass_dir_label != nullptr)
  {
    static const char *dirs[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    int                idx    = (int) ((ui_ctx.compass_heading_deg + 22.5f) / 45.0f) & 0x7;
    sys_ui_widget_set_label_text(w->compass_dir_label, dirs[idx]);
  }

  if (w->compass_needle != nullptr)
  {
    float              rad       = (ui_ctx.compass_heading_deg - 90.0f) * (float) PI / 180.0f;
    int                x2        = SYS_UI_COMPASS_CX + (int) ((SYS_UI_COMPASS_R - 4) * cosf(rad));
    int                y2        = SYS_UI_COMPASS_CY + (int) ((SYS_UI_COMPASS_R - 4) * sinf(rad));
    lv_point_precise_t points[2] = { { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY }, { x2, y2 } };
    lv_line_set_points(w->compass_needle, points, 2);
  }
}

static void sys_ui_drawSettingsOverlay(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->settings_screen == nullptr)
  {
    sys_ui_createSettingsScreen();
  }

  if (w->brightness_slider != nullptr)
  {
    lv_slider_set_value(w->brightness_slider, ui_ctx.brightness_percent, LV_ANIM_OFF);
  }
  if (w->brightness_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->brightness_label, "%d%%", ui_ctx.brightness_percent);
  }

  sys_ui_show_screen(w->settings_screen);
}

static void sys_ui_drawOutOverlay(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->out_screen == nullptr)
  {
    sys_ui_createOutScreen();
  }

  sys_ui_show_screen(w->out_screen);
}

static void sys_ui_drawTimeHistoryOverlay(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->time_history_screen == nullptr)
  {
    sys_ui_createTimeHistoryScreen();
  }

  for (int i = 0; i < 4; ++i)
  {
    if (w->history_labels[i] != nullptr)
    {
      if (i < ui_ctx.rental_history_count)
      {
        sys_ui_widget_set_label_text(w->history_labels[i], ui_ctx.rentalHistory[i]);
      }
      else
      {
        sys_ui_widget_set_label_text(w->history_labels[i], "");
      }
    }
  }

  if (w->time_remaining_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->time_remaining_label, "Remaining: %02d:%02d", ui_ctx.remaining_minutes,
                                        ui_ctx.remaining_seconds);
  }

  sys_ui_show_screen(w->time_history_screen);
}

static void sys_ui_drawDistanceOverlay(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->distance_screen == nullptr)
  {
    sys_ui_createDistanceScreen();
  }

  size_t now   = OS_GET_TICK();
  float  hours = (now - ui_ctx.session_start_ms) / 3600000.0f;
  if (hours < 0.001f)
    hours = 0.001f;
  float avgSpeed = ui_ctx.distance_km / hours;

  if (w->total_distance_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->total_distance_label, "Total: %.2f km", ui_ctx.distance_km);
  }

  if (w->avg_speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(w->avg_speed_label, "Avg speed: %.1f km/h", avgSpeed);
  }

  if (w->distance_chart != nullptr && w->distance_series != nullptr)
  {
    int count = s_distance_history_count;
    int base  = (count > 16) ? (count - 16) : 0;
    int plot  = (count - base) > 0 ? (count - base) : 1;
    lv_chart_set_point_count(w->distance_chart, plot);
    for (int i = 0; i < plot; ++i)
    {
      lv_chart_set_value_by_id(w->distance_chart, w->distance_series, i,
                               (int) (s_distance_history_mock[base + i] * 100));
    }
    lv_chart_refresh(w->distance_chart);
  }

  sys_ui_show_screen(w->distance_screen);
}

static void sys_ui_drawTemperatureOverlay(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->temp_screen == nullptr)
  {
    sys_ui_createTemperatureScreen();
  }

  int count = s_temperature_history_count;
  if (w->temp_chart != nullptr && w->temp_series != nullptr && count > 1)
  {
    int zoom      = ui_ctx.temperature_zoom;
    int panOffset = ui_ctx.temperature_pan;
    int window    = count / zoom;
    if (window < 2)
      window = 2;
    if (window > count)
      window = count;
    if (panOffset < 0)
      panOffset = 0;
    if (panOffset > count - window)
      panOffset = count - window;

    float minTemp = s_temperature_history_mock[panOffset];
    float maxTemp = s_temperature_history_mock[panOffset];
    for (int i = 0; i < window; ++i)
    {
      float v = s_temperature_history_mock[panOffset + i];
      if (v < minTemp)
        minTemp = v;
      if (v > maxTemp)
        maxTemp = v;
    }

    if (w->temp_range_label != nullptr)
    {
      float span = 0;
      if (count > 0)
      {
        span = (s_temperature_time_mock[panOffset + window - 1] - s_temperature_time_mock[panOffset]) / 1000.0f;
      }
      sys_ui_widget_set_label_text_format(w->temp_range_label, "%.1f-%.1fC (%.1fs)", minTemp, maxTemp, span);
    }

    lv_chart_set_point_count(w->temp_chart, window);
    lv_chart_set_range(w->temp_chart, LV_CHART_AXIS_PRIMARY_Y, (int) (minTemp * 10) - 5, (int) (maxTemp * 10) + 5);

    for (int i = 0; i < window; ++i)
    {
      lv_chart_set_value_by_id(w->temp_chart, w->temp_series, i,
                               (int) (s_temperature_history_mock[panOffset + i] * 10));
    }
    lv_chart_refresh(w->temp_chart);
  }

  sys_ui_show_screen(w->temp_screen);
}

static void sys_ui_change_screen(sys_ui_view_t view)
{
  ui_ctx.view = view;
  switch (view)
  {
  case SYS_UI_VIEW_SETTINGS:
  {
    sys_ui_drawSettingsOverlay();
    if (ui_ctx.widgets.settings_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.settings_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.brightness_slider != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.brightness_slider, sys_ui_brightness_slider_cb, LV_EVENT_VALUE_CHANGED,
                          nullptr);
    }
    for (int i = 0; i < 3; ++i)
    {
      if (ui_ctx.widgets.color_btns[i] != nullptr)
      {
        lv_obj_add_event_cb(ui_ctx.widgets.color_btns[i], sys_ui_color_btn_cb, LV_EVENT_CLICKED, (void *) (intptr_t) i);
      }
    }
    break;
  }
  case SYS_UI_VIEW_OUT:
  {
    sys_ui_drawOutOverlay();
    if (ui_ctx.widgets.out_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.out_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    sys_ui_drawTimeHistoryOverlay();
    if (ui_ctx.widgets.time_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.time_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.extend_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.extend_btn, sys_ui_extend_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
    sys_ui_drawDistanceOverlay();
    if (ui_ctx.widgets.distance_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.distance_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_drawTemperatureOverlay();
    if (ui_ctx.widgets.temp_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.temp_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.zoom_minus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.zoom_minus_btn, sys_ui_zoom_minus_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.zoom_plus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.zoom_plus_btn, sys_ui_zoom_plus_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.pan_left_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.pan_left_btn, sys_ui_pan_left_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.pan_right_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.pan_right_btn, sys_ui_pan_right_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  default: break;
  }
}

static void sys_ui_return_main_screen(void)
{
  ui_ctx.view                = SYS_UI_VIEW_MAIN;
  ui_ctx.pending_main_redraw = false;
  sys_ui_init_full_widget();
  sys_ui_register_lvgl_callbacks();
}

static void sys_ui_log_distance_sample(float value)
{
  if (ui_ctx.distance_history_count < SYS_UI_MAX_DISTANCE_LOG)
  {
    ui_ctx.distanceHistory[ui_ctx.distance_history_count++] = value;
    return;
  }

  memmove(&ui_ctx.distanceHistory[0], &ui_ctx.distanceHistory[1],
          (SYS_UI_MAX_DISTANCE_LOG - 1) * sizeof(ui_ctx.distanceHistory[0]));
  ui_ctx.distanceHistory[SYS_UI_MAX_DISTANCE_LOG - 1] = value;
}

static void sys_ui_logTemperatureSample(float value, size_t timestamp)
{
  if (ui_ctx.temperature_sample_count < SYS_UI_MAX_TEMP_SAMPLES)
  {
    ui_ctx.temperatureHistory[ui_ctx.temperature_sample_count]    = value;
    ui_ctx.temperatureTimestamps[ui_ctx.temperature_sample_count] = timestamp;
    ++ui_ctx.temperature_sample_count;
    return;
  }

  memmove(&ui_ctx.temperatureHistory[0], &ui_ctx.temperatureHistory[1],
          (SYS_UI_MAX_TEMP_SAMPLES - 1) * sizeof(ui_ctx.temperatureHistory[0]));
  memmove(&ui_ctx.temperatureTimestamps[0], &ui_ctx.temperatureTimestamps[1],
          (SYS_UI_MAX_TEMP_SAMPLES - 1) * sizeof(ui_ctx.temperatureTimestamps[0]));
  ui_ctx.temperatureHistory[SYS_UI_MAX_TEMP_SAMPLES - 1]    = value;
  ui_ctx.temperatureTimestamps[SYS_UI_MAX_TEMP_SAMPLES - 1] = timestamp;
}

static void sys_ui_refreshTemperatureOverlay(void)
{
  int max_pan = (s_temperature_history_count > 2) ? (s_temperature_history_count - 2) : 0;
  if (ui_ctx.temperature_pan > max_pan)
    ui_ctx.temperature_pan = max_pan;
  if (ui_ctx.temperature_pan < 0)
    ui_ctx.temperature_pan = 0;

  sys_ui_drawTemperatureOverlay();
}

static void sys_ui_init_history_mock(void)
{
  size_t base                 = OS_GET_TICK();
  s_distance_history_count    = 100;
  s_temperature_history_count = 100;
  for (int i = 0; i < 100; ++i)
  {
    s_distance_history_mock[i]    = 0.2f * (float) i;
    s_temperature_history_mock[i] = 26.0f + 2.0f * sinf((float) i / 10.0f);
    s_temperature_time_mock[i]    = base + (size_t) i * 1000U;
  }
}

/* ============== LVGL Callbacks ============== */
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  sys_ui_context_t *ctx = (sys_ui_context_t *) lv_indev_get_user_data(indev);
  if (ctx == nullptr)
  {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  bsp_touch_point_t point = { 0 };
  bsp_touch_read(&point);

  if (point.touched)
  {
    data->point.x = point.x;
    data->point.y = point.y;
    data->state   = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void sys_ui_settings_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Settings button pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_SETTINGS);
}

static void sys_ui_out_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Out button pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_OUT);
}

static void sys_ui_back_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Back button pressed\n");
  sys_ui_return_main_screen();
}

static void sys_ui_extend_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Extend button pressed\n");
  ui_ctx.remaining_minutes += 30;
  sys_ui_drawTimeHistoryOverlay();
}

static void sys_ui_brightness_slider_cb(lv_event_t *e)
{
  lv_obj_t *slider          = (lv_obj_t *) lv_event_get_target(e);
  ui_ctx.brightness_percent = lv_slider_get_value(slider);
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);
  LOG_DBG("[LVGL] Brightness changed to %d%%\n", ui_ctx.brightness_percent);
}

static void sys_ui_color_btn_cb(lv_event_t *e)
{
  int btn_index = (int) (intptr_t) lv_event_get_user_data(e);

  size_t colors[3] = { 0x000000, 0xFFFFFF, 0x7B7B7B };
  if (btn_index >= 0 && btn_index < 3)
  {
    ui_ctx.background_color    = colors[btn_index];
    ui_ctx.pending_main_redraw = true;
    LOG_DBG("[LVGL] Background color changed to index %d\n", btn_index);
  }
}

static void sys_ui_zoom_minus_btn_cb(lv_event_t *e)
{
  (void) e;
  if (ui_ctx.temperature_zoom > 1)
  {
    --ui_ctx.temperature_zoom;
    sys_ui_refreshTemperatureOverlay();
    LOG_DBG("[LVGL] Zoom- pressed, zoom=%d\n", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_zoom_plus_btn_cb(lv_event_t *e)
{
  (void) e;
  if (ui_ctx.temperature_zoom < 4)
  {
    ++ui_ctx.temperature_zoom;
    sys_ui_refreshTemperatureOverlay();
    LOG_DBG("[LVGL] Zoom+ pressed, zoom=%d\n", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_pan_left_btn_cb(lv_event_t *e)
{
  (void) e;
  ui_ctx.temperature_pan -= 5;
  sys_ui_refreshTemperatureOverlay();
  LOG_DBG("[LVGL] Pan left pressed, pan=%d\n", ui_ctx.temperature_pan);
}

static void sys_ui_pan_right_btn_cb(lv_event_t *e)
{
  (void) e;
  ui_ctx.temperature_pan += 5;
  sys_ui_refreshTemperatureOverlay();
  LOG_DBG("[LVGL] Pan right pressed, pan=%d\n", ui_ctx.temperature_pan);
}

static void sys_ui_time_card_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Time card pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_TIME);
}

static void sys_ui_distance_card_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Distance card pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_DISTANCE);
}

static void sys_ui_env_card_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Environment card pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_TEMPERATURE);
}

static void sys_ui_register_lvgl_callbacks(void)
{
  sys_ui_widgets_t *w = &ui_ctx.widgets;

  if (w->settings_btn != nullptr)
  {
    lv_obj_add_event_cb(w->settings_btn, sys_ui_settings_btn_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (w->out_btn != nullptr)
  {
    lv_obj_add_event_cb(w->out_btn, sys_ui_out_btn_cb, LV_EVENT_CLICKED, nullptr);
  }

  if (w->time_card != nullptr)
  {
    sys_ui_widget_set_clickable(w->time_card, sys_ui_time_card_cb, nullptr);
  }
  if (w->distance_card != nullptr)
  {
    sys_ui_widget_set_clickable(w->distance_card, sys_ui_distance_card_cb, nullptr);
  }
  if (w->env_card != nullptr)
  {
    sys_ui_widget_set_clickable(w->env_card, sys_ui_env_card_cb, nullptr);
  }
}

/* End of file -------------------------------------------------------- */