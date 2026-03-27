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

// UI initial colors (can be overridden by settings)
#define SYS_UI_COLOR_BG               SYS_UI_WIDGET_COLOR_BG
#define SYS_UI_COLOR_BG_CARD          SYS_UI_WIDGET_COLOR_BG_CARD
#define SYS_UI_COLOR_PRIMARY          SYS_UI_WIDGET_COLOR_PRIMARY
#define SYS_UI_COLOR_ACCENT           SYS_UI_WIDGET_COLOR_ACCENT
#define SYS_UI_COLOR_SUCCESS          SYS_UI_WIDGET_COLOR_SUCCESS
#define SYS_UI_COLOR_WARNING          SYS_UI_WIDGET_COLOR_WARNING
#define SYS_UI_COLOR_DANGER           SYS_UI_WIDGET_COLOR_DANGER
#define SYS_UI_COLOR_TEXT             SYS_UI_WIDGET_COLOR_TEXT
#define SYS_UI_COLOR_TEXT_DIM         SYS_UI_WIDGET_COLOR_TEXT_DIM

// Timing and data constants
#define SYS_UI_COUNTDOWN_MS           (1000)
#define SYS_UI_ENVIRONMENT_MS         (3000)
#define SYS_UI_MAX_RENTAL_HISTORY     (4)
#define SYS_UI_MAX_DISTANCE_LOG       (16)
#define SYS_UI_MAX_TEMP_SAMPLES       (120)

// Top control buttons
#define SYS_UI_CTRL_BTN_Y             (5)
#define SYS_UI_CTRL_BTN_H             (25)
#define SYS_UI_SETTINGS_BTN_X         (10)
#define SYS_UI_SETTINGS_BTN_W         (90)
#define SYS_UI_OUT_BTN_X              (220)
#define SYS_UI_OUT_BTN_W              (90)

// Speedometer arc
#define SYS_UI_SPEEDO_CX              (107)
#define SYS_UI_SPEEDO_CY              (106)
#define SYS_UI_SPEEDO_OUTER_R         (74)
#define SYS_UI_SPEEDO_INNER_R         (54)

// Map and compass panel
#define SYS_UI_MAP_PANEL_X            (2)
#define SYS_UI_MAP_PANEL_Y            (183)
#define SYS_UI_MAP_PANEL_W            (50)
#define SYS_UI_MAP_PANEL_H            (52)
#define SYS_UI_COMPASS_CX             (22)
#define SYS_UI_COMPASS_CY             (210)
#define SYS_UI_COMPASS_R              (15)
#define SYS_UI_HEADING_TEXT_X         (SYS_UI_COMPASS_CX - 12)
#define SYS_UI_HEADING_TEXT_Y         (SYS_UI_COMPASS_CY - 12)
#define SYS_UI_COMPASS_DEG_X          (SYS_UI_COMPASS_CX - 12)
#define SYS_UI_COMPASS_DEG_Y          (SYS_UI_COMPASS_CY + 5)
#define SYS_UI_COMPASS_PANEL_BG_COLOR (0x0A1520)

/* Right panel info cards */
#define SYS_UI_CARD_X                 (218)
#define SYS_UI_CARD_W                 (100)
#define SYS_UI_TIME_CARD_Y            (34)
#define SYS_UI_TIME_CARD_H            (52)
#define SYS_UI_DIST_CARD_Y            (91)
#define SYS_UI_DIST_CARD_H            (40)
#define SYS_UI_ENV_CARD_Y             (136)
#define SYS_UI_ENV_CARD_H             (100)

/* Sub-screen shared */
#define SYS_UI_BACK_BTN_X             (10)
#define SYS_UI_BACK_BTN_Y             (10)
#define SYS_UI_BACK_BTN_W             (60)
#define SYS_UI_BACK_BTN_H             (25)

/* Settings screen */
#define SYS_UI_SWATCH_ROW_Y           (130)
#define SYS_UI_SWATCH_SIZE            (36)
#define SYS_UI_SWATCH_SPAN            (50)

/* Distance / Temperature sub-screens */
#define SYS_UI_EXTEND_BTN_X           (180)
#define SYS_UI_EXTEND_BTN_Y           (180)
#define SYS_UI_EXTEND_BTN_W           (120)
#define SYS_UI_EXTEND_BTN_H           (30)
#define SYS_UI_TEMP_GRAPH_X           (20)
#define SYS_UI_TEMP_GRAPH_Y           (50)
#define SYS_UI_TEMP_GRAPH_W           (280)
#define SYS_UI_TEMP_GRAPH_H           (140)
#define SYS_UI_TEMP_BTN_Y             (200)
#define SYS_UI_TEMP_BTN_W             (60)
#define SYS_UI_TEMP_BTN_H             (24)
#define SYS_UI_TEMP_BTN_GAP           (10)

/* Background color palette for settings screen
 * INFO(index, R, G, B, label)            */
#define SYS_UI_BG_COLOR_TABLE(INFO) \
  INFO(0, 13, 27, 42, "Navy")       \
  INFO(1, 3, 4, 94, "Ocean")        \
  INFO(2, 10, 10, 10, "Black")
#define SYS_UI_BG_COLOR_COUNT (3)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  lv_obj_t *active_screen;

  // Main screen widgets
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

  // Settings screen widgets
  lv_obj_t *settings_screen;
  lv_obj_t *settings_title;
  lv_obj_t *settings_back_btn;
  lv_obj_t *brightness_slider;
  lv_obj_t *brightness_label;
  lv_obj_t *color_btns[SYS_UI_BG_COLOR_COUNT];

  // Out screen widgets
  lv_obj_t *out_screen;
  lv_obj_t *out_title;
  lv_obj_t *out_back_btn;

  // Time history screen widgets
  lv_obj_t *time_history_screen;
  lv_obj_t *time_history_title;
  lv_obj_t *time_back_btn;
  lv_obj_t *time_remaining_label;
  lv_obj_t *history_labels[4];
  lv_obj_t *extend_btn;

  // Distance screen widgets
  lv_obj_t          *distance_screen;
  lv_obj_t          *distance_title;
  lv_obj_t          *distance_back_btn;
  lv_obj_t          *total_distance_label;
  lv_obj_t          *avg_speed_label;
  lv_obj_t          *distance_chart;
  lv_chart_series_t *distance_series;

  // Temperature screen widgets
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
  lvgl_driver_t    lvgl;
  sys_ui_widgets_t widgets;

  size_t last_speed_update;
  size_t last_second_tick;
  size_t last_time_update_screen;

  size_t            frame_counter;
  float             current_speed;
  float             target_speed;
  float             distance_km;
  int               prev_speed_int;
  sys_fusion_data_t fusion;

  int remaining_minutes;
  int remaining_seconds;

  float temperature_C;
  float humidity;
  int   air_quality;
  int   battery_percent;
  int   brightness_percent;
  float compass_heading_deg;

  size_t background_color;

  size_t        session_start_ms;
  sys_ui_view_t view;
  uint16_t      last_touch_x;
  uint16_t      last_touch_y;
  bool          pending_main_redraw;

  char   rental_history[SYS_UI_MAX_RENTAL_HISTORY][32];
  int    rental_history_count;
  float  distance_history[SYS_UI_MAX_DISTANCE_LOG];
  int    distance_history_count;
  float  temp_history[SYS_UI_MAX_TEMP_SAMPLES];
  size_t temp_timestamps[SYS_UI_MAX_TEMP_SAMPLES];
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
static void sys_ui_init_full_widget(void);
static void sys_ui_reset_full_widget(sys_ui_widgets_t *w);
static void sys_ui_register_lvgl_callbacks(void);

static void sys_ui_update_speed_n_distance(void);
static void sys_ui_update_countdown(void);
static void sys_ui_update_env_data(void);
static void sys_ui_update_speed(int speedKph);
static void sys_ui_update_time(int minutes, int seconds);
static void sys_ui_update_distance(float distance_km);
static void sys_ui_update_env(float temperature_C, float humidity, int air_quality);
static void sys_ui_update_compass(void);

static void sys_ui_change_screen(sys_ui_view_t view);
static void sys_ui_return_main_screen(void);

static void sys_ui_log_distance_sample(float value);
static void sys_ui_log_temp_sample(float value, size_t timestamp);
static void sys_ui_refresh_temp_overlay(void);

static void sys_ui_show_screen(lv_obj_t *screen);
static void sys_ui_create_main_screen(void);
static void sys_ui_create_setting_screen(void);
static void sys_ui_create_out_screen(void);
static void sys_ui_create_time_history_screen(void);
static void sys_ui_create_distance_screen(void);
static void sys_ui_create_temp_screen(void);

static void sys_ui_draw_setting_overlay(void);
static void sys_ui_draw_out_overlay(void);
static void sys_ui_draw_time_overlay(void);
static void sys_ui_draw_distance_overlay(void);
static void sys_ui_draw_temp_overlay(void);
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

  sys_ui_reset_full_widget(&ui_ctx.widgets);
  bsp_display_init();

  ui_ctx.lvgl.user_data = &ui_ctx;
  lvgl_driver_init(&ui_ctx.lvgl, bsp_display_get_driver());
  lvgl_driver_set_touch_callback(&ui_ctx.lvgl, sys_ui_lvgl_touch_read_cb);

  sys_ui_init_data();
  sys_ui_init_history_mock();
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);
  sys_ui_init_full_widget();
  sys_ui_register_lvgl_callbacks();

  size_t now                     = OS_GET_TICK();
  ui_ctx.session_start_ms        = now;
  ui_ctx.last_speed_update       = now;
  ui_ctx.last_second_tick        = now;
  ui_ctx.last_time_update_screen = now;

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
      sys_ui_draw_time_overlay();
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    sys_ui_draw_distance_overlay();
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_draw_temp_overlay();
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

  if (ui_ctx.view == SYS_UI_VIEW_MAIN && ui_ctx.pending_main_redraw)
  {
    if (ui_ctx.widgets.main_screen != nullptr)
    {
      lv_obj_del(ui_ctx.widgets.main_screen);
    }
    sys_ui_reset_full_widget(&ui_ctx.widgets);
    sys_ui_init_full_widget();
    sys_ui_register_lvgl_callbacks();
    ui_ctx.pending_main_redraw = false;
  }

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
  for (int i = 0; i < SYS_UI_BG_COLOR_COUNT; ++i) w->color_btns[i] = nullptr;
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
    strncpy(ui_ctx.rental_history[i], seedHistory[i], sizeof(ui_ctx.rental_history[i]) - 1);
    ui_ctx.rental_history[i][sizeof(ui_ctx.rental_history[i]) - 1] = '\0';
  }
}

static void sys_ui_show_screen(lv_obj_t *screen)
{
  if (screen == nullptr)
    return;

  ui_ctx.widgets.active_screen = screen;
  lv_screen_load(screen);
}

static void sys_ui_create_main_screen(void)
{
  ui_ctx.widgets.main_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  // Top control buttons
  ui_ctx.widgets.settings_btn = sys_ui_widget_create_button(ui_ctx.widgets.main_screen, SYS_UI_SETTINGS_BTN_X,
                                                            SYS_UI_CTRL_BTN_Y, SYS_UI_SETTINGS_BTN_W, SYS_UI_CTRL_BTN_H,
                                                            "SETTINGS", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);

  ui_ctx.widgets.out_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.main_screen, SYS_UI_OUT_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_OUT_BTN_W,
                                SYS_UI_CTRL_BTN_H, "OUT", SYS_UI_COLOR_WARNING, SYS_UI_COLOR_BG);

  // Speedometer Arc (center)
  ui_ctx.widgets.speedometer_arc =
    sys_ui_widget_create_arc(ui_ctx.widgets.main_screen, SYS_UI_SPEEDO_CX, SYS_UI_SPEEDO_CY, SYS_UI_SPEEDO_OUTER_R, 135,
                             405, 0x1A2A3A, SYS_UI_COLOR_SUCCESS);

  ui_ctx.widgets.speed_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_SPEEDO_CX - 20, SYS_UI_SPEEDO_CY - 14, "0",
                               SYS_UI_COLOR_TEXT, &lv_font_montserrat_28);
  lv_obj_set_width(ui_ctx.widgets.speed_label, 40);
  lv_obj_set_style_text_align(ui_ctx.widgets.speed_label, LV_TEXT_ALIGN_CENTER, 0);

  ui_ctx.widgets.speed_unit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_SPEEDO_CX - 12, SYS_UI_SPEEDO_CY + 18, "km/h",
                               SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);

  // Map and compass panel
  sys_ui_widget_create_panel(ui_ctx.widgets.main_screen, SYS_UI_MAP_PANEL_X, SYS_UI_MAP_PANEL_Y, SYS_UI_MAP_PANEL_W,
                             SYS_UI_MAP_PANEL_H, SYS_UI_COMPASS_PANEL_BG_COLOR);

  static lv_point_precise_t needle_points[2] = { { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY },
                                                 { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY - SYS_UI_COMPASS_R + 4 } };
  ui_ctx.widgets.compass_needle              = lv_line_create(ui_ctx.widgets.main_screen);
  lv_line_set_points(ui_ctx.widgets.compass_needle, needle_points, 2);
  lv_obj_set_style_line_width(ui_ctx.widgets.compass_needle, 2, 0);
  lv_obj_set_style_line_color(ui_ctx.widgets.compass_needle, lv_color_hex(SYS_UI_COLOR_ACCENT), 0);

  ui_ctx.widgets.compass_deg_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_HEADING_TEXT_X, SYS_UI_HEADING_TEXT_Y, "0°",
                               SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);
  ui_ctx.widgets.compass_dir_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_COMPASS_DEG_X, SYS_UI_COMPASS_DEG_Y, "N",
                               SYS_UI_COLOR_ACCENT, &lv_font_montserrat_18);

  // Right Panel Info Cards

  // Time Card
  ui_ctx.widgets.time_card = sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_TIME_CARD_Y,
                                                       SYS_UI_CARD_W, SYS_UI_TIME_CARD_H, SYS_UI_COLOR_PRIMARY);
  sys_ui_widget_create_label(ui_ctx.widgets.time_card, 8, 2, "REMAIN", SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);
  ui_ctx.widgets.time_label =
    sys_ui_widget_create_label(ui_ctx.widgets.time_card, 5, 16, "00:00", SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_18);
  ui_ctx.widgets.time_unit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.time_card, 24, 38, "mins", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  /* Distance Card */
  ui_ctx.widgets.distance_card =
    sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_DIST_CARD_Y, SYS_UI_CARD_W,
                              SYS_UI_DIST_CARD_H, SYS_UI_COLOR_ACCENT);
  sys_ui_widget_create_label(ui_ctx.widgets.distance_card, 5, 2, "DISTANCE", SYS_UI_COLOR_TEXT_DIM,
                             &lv_font_montserrat_10);
  ui_ctx.widgets.distance_label      = sys_ui_widget_create_label(ui_ctx.widgets.distance_card, 5, 16, "0.00",
                                                                  SYS_UI_COLOR_ACCENT, &lv_font_montserrat_16);
  ui_ctx.widgets.distance_unit_label = sys_ui_widget_create_label(ui_ctx.widgets.distance_card, 62, 20, "km",
                                                                  SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Environment Card
  ui_ctx.widgets.env_card = sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_ENV_CARD_Y,
                                                      SYS_UI_CARD_W, SYS_UI_ENV_CARD_H, SYS_UI_COLOR_TEXT_DIM);
  sys_ui_widget_create_label(ui_ctx.widgets.env_card, 6, 0, "ENV STATUS", SYS_UI_COLOR_TEXT, &lv_font_montserrat_10);
  ui_ctx.widgets.temp_label = sys_ui_widget_create_label(ui_ctx.widgets.env_card, 5, 14, "0.0\xc2\xb0\x43",
                                                         SYS_UI_COLOR_WARNING, &lv_font_montserrat_12);
  ui_ctx.widgets.humidity_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_card, 5, 32, "0%", SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_12);
  ui_ctx.widgets.aqi_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_card, 5, 50, "AQI: --", SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_10);
}

static void sys_ui_create_setting_screen(void)
{
  ui_ctx.widgets.settings_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.settings_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.settings_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "< BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  ui_ctx.widgets.settings_title = sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 90, 13, "SETTINGS",
                                                             SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  /* Brightness section */
  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 40, 52, "Brightness", SYS_UI_COLOR_TEXT_DIM, nullptr);

  ui_ctx.widgets.brightness_slider = sys_ui_widget_create_slider(
    ui_ctx.widgets.settings_screen, 40, 72, 220, 22, 5, 100, ui_ctx.brightness_percent, 0x1A2A3A, SYS_UI_COLOR_PRIMARY);

  ui_ctx.widgets.brightness_label =
    sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 268, 72, "", SYS_UI_COLOR_TEXT, nullptr);
  sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, "%d%%", ui_ctx.brightness_percent);

  /* Background color section */
  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 40, 108, "Background", SYS_UI_COLOR_TEXT_DIM, nullptr);

  /* Color swatches — generated from BG_COLOR_TABLE */
  // clang-format off
#define INFO(idx, r, g, b, lbl)                                                                          \
  ui_ctx.widgets.color_btns[idx] = lv_btn_create(ui_ctx.widgets.settings_screen);                       \
  lv_obj_set_pos(ui_ctx.widgets.color_btns[idx], 40 + (idx) * SYS_UI_SWATCH_SPAN, SYS_UI_SWATCH_ROW_Y);  \
  lv_obj_set_size(ui_ctx.widgets.color_btns[idx], SYS_UI_SWATCH_SIZE, SYS_UI_SWATCH_SIZE);               \
  lv_obj_set_style_bg_color(ui_ctx.widgets.color_btns[idx], lv_color_hex(BSP_DISPLAY_RGB_TO_HEX(r, g, b)), 0); \
  lv_obj_set_style_border_color(ui_ctx.widgets.color_btns[idx], lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0); \
  lv_obj_set_style_border_width(ui_ctx.widgets.color_btns[idx], 1, 0);                                   \
  lv_obj_set_style_radius(ui_ctx.widgets.color_btns[idx], 4, 0);                                         \
  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen,                                            \
    40 + (idx) * SYS_UI_SWATCH_SPAN, SYS_UI_SWATCH_ROW_Y + SYS_UI_SWATCH_SIZE + 3,                      \
    lbl, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  SYS_UI_BG_COLOR_TABLE(INFO)
#undef INFO
  // clang-format on
}

static void sys_ui_create_out_screen(void)
{
  ui_ctx.widgets.out_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.out_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.out_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  ui_ctx.widgets.out_title = sys_ui_widget_create_label(ui_ctx.widgets.out_screen, 80, 80, "OUT MODE",
                                                        SYS_UI_COLOR_WARNING, &lv_font_montserrat_24);

  sys_ui_widget_create_label(ui_ctx.widgets.out_screen, 80, 130, "Tap BACK to resume.", SYS_UI_COLOR_TEXT, nullptr);
}

static void sys_ui_create_time_history_screen(void)
{
  ui_ctx.widgets.time_history_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.time_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.time_history_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y,
                                SYS_UI_BACK_BTN_W, SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  ui_ctx.widgets.time_history_title = sys_ui_widget_create_label(
    ui_ctx.widgets.time_history_screen, 60, 15, "RENTAL HISTORY", SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  for (int i = 0; i < 4; ++i)
  {
    ui_ctx.widgets.history_labels[i] = sys_ui_widget_create_label(ui_ctx.widgets.time_history_screen, 40, 55 + i * 20,
                                                                  "", SYS_UI_COLOR_TEXT, &lv_font_montserrat_12);
  }

  ui_ctx.widgets.time_remaining_label = sys_ui_widget_create_label(ui_ctx.widgets.time_history_screen, 40, 140,
                                                                   "Remaining: 00:00", SYS_UI_COLOR_TEXT, nullptr);

  ui_ctx.widgets.extend_btn = sys_ui_widget_create_button(ui_ctx.widgets.time_history_screen, SYS_UI_EXTEND_BTN_X,
                                                          SYS_UI_EXTEND_BTN_Y, SYS_UI_EXTEND_BTN_W, SYS_UI_EXTEND_BTN_H,
                                                          "+30 min", SYS_UI_COLOR_SUCCESS, SYS_UI_COLOR_BG);
}

static void sys_ui_create_distance_screen(void)
{
  ui_ctx.widgets.distance_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.distance_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.distance_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  ui_ctx.widgets.distance_title = sys_ui_widget_create_label(ui_ctx.widgets.distance_screen, 70, 15, "DISTANCE STATS",
                                                             SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  ui_ctx.widgets.total_distance_label =
    sys_ui_widget_create_label(ui_ctx.widgets.distance_screen, 30, 50, "Total: 0.00 km", SYS_UI_COLOR_TEXT, nullptr);

  ui_ctx.widgets.avg_speed_label = sys_ui_widget_create_label(ui_ctx.widgets.distance_screen, 30, 70,
                                                              "Avg speed: 0.0 km/h", SYS_UI_COLOR_TEXT, nullptr);

  ui_ctx.widgets.distance_chart =
    sys_ui_widget_create_chart(ui_ctx.widgets.distance_screen, 30, 95, 260, 120, 16, 0x111111);
  ui_ctx.widgets.distance_series = sys_ui_widget_addChart_series(ui_ctx.widgets.distance_chart, SYS_UI_COLOR_ACCENT);
}

static void sys_ui_create_temp_screen(void)
{
  ui_ctx.widgets.temp_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.temp_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.temp_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  ui_ctx.widgets.temp_title = sys_ui_widget_create_label(ui_ctx.widgets.temp_screen, 90, 15, "TEMPERATURE",
                                                         SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  ui_ctx.widgets.temp_chart =
    sys_ui_widget_create_chart(ui_ctx.widgets.temp_screen, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_GRAPH_Y,
                               SYS_UI_TEMP_GRAPH_W, SYS_UI_TEMP_GRAPH_H, 60, 0x111111);
  ui_ctx.widgets.temp_series = sys_ui_widget_addChart_series(ui_ctx.widgets.temp_chart, SYS_UI_COLOR_ACCENT);

  ui_ctx.widgets.temp_range_label =
    sys_ui_widget_create_label(ui_ctx.widgets.temp_screen, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_GRAPH_Y - 15,
                               "Collecting...", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  ui_ctx.widgets.zoom_minus_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.temp_screen, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_BTN_Y, SYS_UI_TEMP_BTN_W,
                                SYS_UI_TEMP_BTN_H, "Zoom-", 0x333333, SYS_UI_COLOR_TEXT);

  ui_ctx.widgets.zoom_plus_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.temp_screen, SYS_UI_TEMP_GRAPH_X + SYS_UI_TEMP_BTN_W + SYS_UI_TEMP_BTN_GAP, SYS_UI_TEMP_BTN_Y,
    SYS_UI_TEMP_BTN_W, SYS_UI_TEMP_BTN_H, "Zoom+", SYS_UI_COLOR_SUCCESS, SYS_UI_COLOR_BG);

  ui_ctx.widgets.pan_left_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.temp_screen, SYS_UI_TEMP_GRAPH_X + 2 * (SYS_UI_TEMP_BTN_W + SYS_UI_TEMP_BTN_GAP), SYS_UI_TEMP_BTN_Y,
    SYS_UI_TEMP_BTN_W, SYS_UI_TEMP_BTN_H, "<", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);

  ui_ctx.widgets.pan_right_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.temp_screen, SYS_UI_TEMP_GRAPH_X + 3 * (SYS_UI_TEMP_BTN_W + SYS_UI_TEMP_BTN_GAP), SYS_UI_TEMP_BTN_Y,
    SYS_UI_TEMP_BTN_W, SYS_UI_TEMP_BTN_H, ">", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);
}

static void sys_ui_init_full_widget(void)
{
  if (ui_ctx.widgets.main_screen == nullptr)
  {
    sys_ui_create_main_screen();
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

  if (ui_ctx.widgets.speedometer_arc != nullptr)
  {
    int arcValue = (speedKph * 100) / 40;
    sys_ui_widget_set_arc_value(ui_ctx.widgets.speedometer_arc, arcValue);

    size_t arcColor = SYS_UI_COLOR_DANGER;
    if (speedKph <= 20)
      arcColor = SYS_UI_COLOR_SUCCESS;
    else if (speedKph <= 30)
      arcColor = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_arc_color(ui_ctx.widgets.speedometer_arc, arcColor);
  }

  if (ui_ctx.widgets.speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.speed_label, "%d", speedKph);
  }
}

static void sys_ui_update_time(int minutes, int seconds)
{
  if (ui_ctx.widgets.time_label == nullptr)
    return;

  sys_ui_widget_set_label_text_format(ui_ctx.widgets.time_label, "%02d:%02d", minutes, seconds);

  size_t timeColor = SYS_UI_COLOR_SUCCESS;
  if (minutes < 5)
    timeColor = SYS_UI_COLOR_DANGER;
  else if (minutes < 10)
    timeColor = SYS_UI_COLOR_WARNING;
  sys_ui_widget_set_label_color(ui_ctx.widgets.time_label, timeColor);
}

static void sys_ui_update_distance(float distance_km)
{
  if (ui_ctx.widgets.distance_label == nullptr)
    return;

  sys_ui_widget_set_label_text_format(ui_ctx.widgets.distance_label, "%.2f", distance_km);
}

static void sys_ui_update_env(float temperature_C, float humidity, int air_quality)
{
  if (ui_ctx.widgets.temp_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.temp_label, "%.1f°C", temperature_C);
  }

  if (ui_ctx.widgets.humidity_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.humidity_label, "%.0f%%", humidity);
  }

  if (ui_ctx.widgets.aqi_label != nullptr)
  {
    const char *status = (air_quality >= 80) ? "Good" : ((air_quality >= 50) ? "Fair" : "Poor");
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.aqi_label, "AQI:%d %s", air_quality, status);

    size_t aqiColor = SYS_UI_COLOR_DANGER;
    if (air_quality >= 80)
      aqiColor = SYS_UI_COLOR_SUCCESS;
    else if (air_quality >= 50)
      aqiColor = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_label_color(ui_ctx.widgets.aqi_label, aqiColor);
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

  sys_ui_log_temp_sample(ui_ctx.temperature_C, OS_GET_TICK());
}

static void sys_ui_update_compass(void)
{
  if (ui_ctx.widgets.compass_deg_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.compass_deg_label, "%.0f°", ui_ctx.compass_heading_deg);
  }

  if (ui_ctx.widgets.compass_dir_label != nullptr)
  {
    static const char *dirs[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    int                idx    = (int) ((ui_ctx.compass_heading_deg + 22.5f) / 45.0f) & 0x7;
    sys_ui_widget_set_label_text(ui_ctx.widgets.compass_dir_label, dirs[idx]);
  }

  if (ui_ctx.widgets.compass_needle != nullptr)
  {
    float              rad       = (ui_ctx.compass_heading_deg - 90.0f) * (float) PI / 180.0f;
    int                x2        = SYS_UI_COMPASS_CX + (int) ((SYS_UI_COMPASS_R - 4) * cosf(rad));
    int                y2        = SYS_UI_COMPASS_CY + (int) ((SYS_UI_COMPASS_R - 4) * sinf(rad));
    lv_point_precise_t points[2] = { { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY }, { x2, y2 } };
    lv_line_set_points(ui_ctx.widgets.compass_needle, points, 2);
  }
}

static void sys_ui_draw_setting_overlay(void)
{
  if (ui_ctx.widgets.settings_screen == nullptr)
  {
    sys_ui_create_setting_screen();
  }

  if (ui_ctx.widgets.brightness_slider != nullptr)
  {
    lv_slider_set_value(ui_ctx.widgets.brightness_slider, ui_ctx.brightness_percent, LV_ANIM_OFF);
  }
  if (ui_ctx.widgets.brightness_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, "%d%%", ui_ctx.brightness_percent);
  }

  sys_ui_show_screen(ui_ctx.widgets.settings_screen);
}

static void sys_ui_draw_out_overlay(void)
{
  if (ui_ctx.widgets.out_screen == nullptr)
  {
    sys_ui_create_out_screen();
  }

  sys_ui_show_screen(ui_ctx.widgets.out_screen);
}

static void sys_ui_draw_time_overlay(void)
{
  if (ui_ctx.widgets.time_history_screen == nullptr)
  {
    sys_ui_create_time_history_screen();
  }

  for (int i = 0; i < 4; ++i)
  {
    if (ui_ctx.widgets.history_labels[i] != nullptr)
    {
      if (i < ui_ctx.rental_history_count)
      {
        sys_ui_widget_set_label_text(ui_ctx.widgets.history_labels[i], ui_ctx.rental_history[i]);
      }
      else
      {
        sys_ui_widget_set_label_text(ui_ctx.widgets.history_labels[i], "");
      }
    }
  }

  if (ui_ctx.widgets.time_remaining_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.time_remaining_label, "Remaining: %02d:%02d",
                                        ui_ctx.remaining_minutes, ui_ctx.remaining_seconds);
  }

  sys_ui_show_screen(ui_ctx.widgets.time_history_screen);
}

static void sys_ui_draw_distance_overlay(void)
{
  if (ui_ctx.widgets.distance_screen == nullptr)
  {
    sys_ui_create_distance_screen();
  }

  size_t now   = OS_GET_TICK();
  float  hours = (now - ui_ctx.session_start_ms) / 3600000.0f;
  if (hours < 0.001f)
    hours = 0.001f;
  float avgSpeed = ui_ctx.distance_km / hours;

  if (ui_ctx.widgets.total_distance_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.total_distance_label, "Total: %.2f km", ui_ctx.distance_km);
  }

  if (ui_ctx.widgets.avg_speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.avg_speed_label, "Avg speed: %.1f km/h", avgSpeed);
  }

  if (ui_ctx.widgets.distance_chart != nullptr && ui_ctx.widgets.distance_series != nullptr)
  {
    int count = s_distance_history_count;
    int base  = (count > 16) ? (count - 16) : 0;
    int plot  = (count - base) > 0 ? (count - base) : 1;
    lv_chart_set_point_count(ui_ctx.widgets.distance_chart, plot);
    for (int i = 0; i < plot; ++i)
    {
      lv_chart_set_value_by_id(ui_ctx.widgets.distance_chart, ui_ctx.widgets.distance_series, i,
                               (int) (s_distance_history_mock[base + i] * 100));
    }
    lv_chart_refresh(ui_ctx.widgets.distance_chart);
  }

  sys_ui_show_screen(ui_ctx.widgets.distance_screen);
}

static void sys_ui_draw_temp_overlay(void)
{
  if (ui_ctx.widgets.temp_screen == nullptr)
  {
    sys_ui_create_temp_screen();
  }

  int count = s_temperature_history_count;
  if (ui_ctx.widgets.temp_chart != nullptr && ui_ctx.widgets.temp_series != nullptr && count > 1)
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

    if (ui_ctx.widgets.temp_range_label != nullptr)
    {
      float span = 0;
      if (count > 0)
      {
        span = (s_temperature_time_mock[panOffset + window - 1] - s_temperature_time_mock[panOffset]) / 1000.0f;
      }
      sys_ui_widget_set_label_text_format(ui_ctx.widgets.temp_range_label, "%.1f-%.1fC (%.1fs)", minTemp, maxTemp,
                                          span);
    }

    lv_chart_set_point_count(ui_ctx.widgets.temp_chart, window);
    lv_chart_set_range(ui_ctx.widgets.temp_chart, LV_CHART_AXIS_PRIMARY_Y, (int) (minTemp * 10) - 5,
                       (int) (maxTemp * 10) + 5);

    for (int i = 0; i < window; ++i)
    {
      lv_chart_set_value_by_id(ui_ctx.widgets.temp_chart, ui_ctx.widgets.temp_series, i,
                               (int) (s_temperature_history_mock[panOffset + i] * 10));
    }
    lv_chart_refresh(ui_ctx.widgets.temp_chart);
  }

  sys_ui_show_screen(ui_ctx.widgets.temp_screen);
}

static void sys_ui_change_screen(sys_ui_view_t view)
{
  ui_ctx.view = view;
  switch (view)
  {
  case SYS_UI_VIEW_SETTINGS:
  {
    sys_ui_draw_setting_overlay();
    if (ui_ctx.widgets.settings_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.settings_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.brightness_slider != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.brightness_slider, sys_ui_brightness_slider_cb, LV_EVENT_VALUE_CHANGED,
                          nullptr);
    }
    for (int i = 0; i < SYS_UI_BG_COLOR_COUNT; ++i)
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
    sys_ui_draw_out_overlay();
    if (ui_ctx.widgets.out_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.out_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    sys_ui_draw_time_overlay();
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
    sys_ui_draw_distance_overlay();
    if (ui_ctx.widgets.distance_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.distance_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_draw_temp_overlay();
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
  ui_ctx.view = SYS_UI_VIEW_MAIN;

  if (ui_ctx.pending_main_redraw)
  {
    if (ui_ctx.widgets.main_screen != nullptr)
    {
      lv_obj_del(ui_ctx.widgets.main_screen);
    }
    if (ui_ctx.widgets.settings_screen != nullptr)
    {
      lv_obj_del(ui_ctx.widgets.settings_screen);
    }
    sys_ui_reset_full_widget(&ui_ctx.widgets);
  }

  ui_ctx.pending_main_redraw = false;
  sys_ui_init_full_widget();
  sys_ui_register_lvgl_callbacks();
}

static void sys_ui_log_distance_sample(float value)
{
  if (ui_ctx.distance_history_count < SYS_UI_MAX_DISTANCE_LOG)
  {
    ui_ctx.distance_history[ui_ctx.distance_history_count++] = value;
    return;
  }

  memmove(&ui_ctx.distance_history[0], &ui_ctx.distance_history[1],
          (SYS_UI_MAX_DISTANCE_LOG - 1) * sizeof(ui_ctx.distance_history[0]));
  ui_ctx.distance_history[SYS_UI_MAX_DISTANCE_LOG - 1] = value;
}

static void sys_ui_log_temp_sample(float value, size_t timestamp)
{
  if (ui_ctx.temperature_sample_count < SYS_UI_MAX_TEMP_SAMPLES)
  {
    ui_ctx.temp_history[ui_ctx.temperature_sample_count]    = value;
    ui_ctx.temp_timestamps[ui_ctx.temperature_sample_count] = timestamp;
    ++ui_ctx.temperature_sample_count;
    return;
  }

  memmove(&ui_ctx.temp_history[0], &ui_ctx.temp_history[1],
          (SYS_UI_MAX_TEMP_SAMPLES - 1) * sizeof(ui_ctx.temp_history[0]));
  memmove(&ui_ctx.temp_timestamps[0], &ui_ctx.temp_timestamps[1],
          (SYS_UI_MAX_TEMP_SAMPLES - 1) * sizeof(ui_ctx.temp_timestamps[0]));
  ui_ctx.temp_history[SYS_UI_MAX_TEMP_SAMPLES - 1]    = value;
  ui_ctx.temp_timestamps[SYS_UI_MAX_TEMP_SAMPLES - 1] = timestamp;
}

static void sys_ui_refresh_temp_overlay(void)
{
  int max_pan = (s_temperature_history_count > 2) ? (s_temperature_history_count - 2) : 0;
  if (ui_ctx.temperature_pan > max_pan)
    ui_ctx.temperature_pan = max_pan;
  if (ui_ctx.temperature_pan < 0)
    ui_ctx.temperature_pan = 0;

  sys_ui_draw_temp_overlay();
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

//================= LVGL callbacks and event handlers =========================
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

static void sys_ui_settings_btn_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Settings button pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_SETTINGS);
}

static void sys_ui_out_btn_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Out button pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_OUT);
}

static void sys_ui_back_btn_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Back button pressed\n");
  sys_ui_return_main_screen();
}

static void sys_ui_extend_btn_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Extend button pressed\n");
  ui_ctx.remaining_minutes += 30;
  sys_ui_draw_time_overlay();
}

static void sys_ui_brightness_slider_cb(lv_event_t *event)
{
  lv_obj_t *slider          = (lv_obj_t *) lv_event_get_target(event);
  ui_ctx.brightness_percent = lv_slider_get_value(slider);
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);

  if (ui_ctx.widgets.brightness_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, "%d%%", ui_ctx.brightness_percent);
  }

  LOG_DBG("[LVGL] Brightness changed to %d%%\n", ui_ctx.brightness_percent);
}

static void sys_ui_color_btn_cb(lv_event_t *event)
{
  int btn_index = (int) (intptr_t) lv_event_get_user_data(event);

#define INFO(idx, r, g, b, lbl) BSP_DISPLAY_RGB_TO_HEX(r, g, b),
  static const size_t bg_colors[SYS_UI_BG_COLOR_COUNT] = { SYS_UI_BG_COLOR_TABLE(INFO) };
#undef INFO

  if (btn_index >= 0 && btn_index < SYS_UI_BG_COLOR_COUNT)
  {
    ui_ctx.background_color    = bg_colors[btn_index];
    ui_ctx.pending_main_redraw = true;
    LOG_DBG("[LVGL] Background color changed to index %d\n", btn_index);
  }
}

static void sys_ui_zoom_minus_btn_cb(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.temperature_zoom > 1)
  {
    --ui_ctx.temperature_zoom;
    sys_ui_refresh_temp_overlay();
    LOG_DBG("[LVGL] Zoom- pressed, zoom=%d\n", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_zoom_plus_btn_cb(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.temperature_zoom < 4)
  {
    ++ui_ctx.temperature_zoom;
    sys_ui_refresh_temp_overlay();
    LOG_DBG("[LVGL] Zoom+ pressed, zoom=%d\n", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_pan_left_btn_cb(lv_event_t *event)
{
  (void) event;
  ui_ctx.temperature_pan -= 5;
  sys_ui_refresh_temp_overlay();
  LOG_DBG("[LVGL] Pan left pressed, pan=%d\n", ui_ctx.temperature_pan);
}

static void sys_ui_pan_right_btn_cb(lv_event_t *event)
{
  (void) event;
  ui_ctx.temperature_pan += 5;
  sys_ui_refresh_temp_overlay();
  LOG_DBG("[LVGL] Pan right pressed, pan=%d\n", ui_ctx.temperature_pan);
}

static void sys_ui_time_card_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Time card pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_TIME);
}

static void sys_ui_distance_card_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Distance card pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_DISTANCE);
}

static void sys_ui_env_card_cb(lv_event_t *event)
{
  (void) event;
  LOG_DBG("[LVGL] Environment card pressed\n");
  sys_ui_change_screen(SYS_UI_VIEW_TEMPERATURE);
}

static void sys_ui_register_lvgl_callbacks(void)
{
  if (ui_ctx.widgets.settings_btn != nullptr)
  {
    lv_obj_add_event_cb(ui_ctx.widgets.settings_btn, sys_ui_settings_btn_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (ui_ctx.widgets.out_btn != nullptr)
  {
    lv_obj_add_event_cb(ui_ctx.widgets.out_btn, sys_ui_out_btn_cb, LV_EVENT_CLICKED, nullptr);
  }

  if (ui_ctx.widgets.time_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.time_card, sys_ui_time_card_cb, nullptr);
  }
  if (ui_ctx.widgets.distance_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.distance_card, sys_ui_distance_card_cb, nullptr);
  }
  if (ui_ctx.widgets.env_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.env_card, sys_ui_env_card_cb, nullptr);
  }
}

/* End of file -------------------------------------------------------- */