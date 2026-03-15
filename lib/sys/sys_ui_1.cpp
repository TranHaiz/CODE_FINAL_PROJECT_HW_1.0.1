/**
 * @file       sys_ui.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Full UI Layer)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      UI controller with all LVGL widget creation and event handling.
 *             bsp_display only provides hardware access; all UI logic is here.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_ui_1.h"

#include "log_service.h"
#include "os_lib.h"

#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_ui, LOG_LEVEL_DBG);
#define SYS_UI_CLAMP(val, minv, maxv) ((val) < (minv) ? (minv) : ((val) > (maxv) ? (maxv) : (val)))

/* Private enumerate/structure ---------------------------------------- */
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
  size_t frame_counter;
  float  current_speed;
  float  target_speed;
  float  distance_km;
  int    prev_speed_int;

  /* Countdown timer */
  int remaining_minutes;
  int remaining_seconds;

  /* Environment telemetry */
  float temperature_C;
  float humidity;
  int   air_quality;
  int   battery_percent;
  int   brightness_percent;

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
} sys_ui_t;

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
sys_ui_t ui_ctx;

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

static void sys_ui_init_data(sys_ui_t *ctx);
static void sys_ui_reset_full_widget(sys_ui_widgets_t *w);
static void sys_ui_update_speed_n_distance(sys_ui_t *ctx);
static void sys_ui_update_countdown(sys_ui_t *ctx);
static void sys_ui_update_env_data(sys_ui_t *ctx);
static void sys_ui_enterView(sys_ui_t *ctx, sys_ui_view_t view);
static void sys_ui_returnToMain(sys_ui_t *ctx);
static void sys_ui_logDistanceSample(sys_ui_t *ctx, float value);
static void sys_ui_logTemperatureSample(sys_ui_t *ctx, float value, size_t timestamp);
static void sys_ui_refreshTemperatureOverlay(sys_ui_t *ctx);
static void sys_ui_register_lvgl_callbacks(sys_ui_t *ctx);
static void sys_ui_show_screen(sys_ui_t *ctx, lv_obj_t *screen);
static void sys_ui_main_screen(sys_ui_t *ctx);
static void sys_ui_createSettingsScreen(sys_ui_t *ctx);
static void sys_ui_createOutScreen(sys_ui_t *ctx);
static void sys_ui_createTimeHistoryScreen(sys_ui_t *ctx);
static void sys_ui_createDistanceScreen(sys_ui_t *ctx);
static void sys_ui_createTemperatureScreen(sys_ui_t *ctx);
static void sys_ui_init_full_widget(sys_ui_t *ctx);
static void sys_ui_update_speed(sys_ui_t *ctx, int speedKph);
static void sys_ui_update_time(sys_ui_t *ctx, int minutes, int seconds);
static void sys_ui_update_distance(sys_ui_t *ctx, float distance_km);
static void sys_ui_update_env(sys_ui_t *ctx, float temperature_C, float humidity, int air_quality);
static void sys_ui_drawSettingsOverlay(sys_ui_t *ctx);
static void sys_ui_drawOutOverlay(sys_ui_t *ctx);
static void sys_ui_drawTimeHistoryOverlay(sys_ui_t *ctx);
static void sys_ui_drawDistanceOverlay(sys_ui_t *ctx);
static void sys_ui_drawTemperatureOverlay(sys_ui_t *ctx);

/* Function definitions ----------------------------------------------- */
void sys_ui_init(void)
{
  ui_ctx.last_speed_update        = 0;
  ui_ctx.last_second_tick         = 0;
  ui_ctx.last_time_update_screen  = 0;
  ui_ctx.frame_counter            = 0;
  ui_ctx.current_speed            = 0.0f;
  ui_ctx.target_speed             = 0.0f;
  ui_ctx.distance_km              = 0.0f;
  ui_ctx.prev_speed_int           = -1;
  ui_ctx.remaining_minutes        = 0;
  ui_ctx.remaining_seconds        = 0;
  ui_ctx.temperature_C            = 0.0f;
  ui_ctx.humidity                 = 0.0f;
  ui_ctx.air_quality              = 0;
  ui_ctx.battery_percent          = 85;
  ui_ctx.brightness_percent       = 80;
  ui_ctx.background_color         = SYS_UI_COLOR_BG;
  ui_ctx.session_start_ms         = OS_GET_TICK();
  ui_ctx.view                     = SYS_UI_VIEW_MAIN;
  ui_ctx.last_touch_x             = 0;
  ui_ctx.last_touch_y             = 0;
  ui_ctx.pending_main_redraw      = false;
  ui_ctx.rental_history_count     = 0;
  ui_ctx.distance_history_count   = 0;
  ui_ctx.temperature_sample_count = 0;
  ui_ctx.temperature_zoom         = 1;
  ui_ctx.temperature_pan          = 0;

  /* Initialize widget pointers */
  sys_ui_reset_full_widget(&ui_ctx.widgets);

  // Int first display
  bsp_display_init();

  // Initialize LVGL driver
  ui_ctx.lvgl.user_data = &ui_ctx;
  lvgl_driver_init(&ui_ctx.lvgl, bsp_display_get_driver());
  lvgl_driver_set_touch_callback(&ui_ctx.lvgl, sys_ui_lvgl_touch_read_cb);

  // Init UI data (speed, time, env, history)
  sys_ui_init_data(&ui_ctx);

  /* Set brightness */
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);

  /* Create and show main screen */
  sys_ui_init_full_widget(&ui_ctx);

  /* Register LVGL button event callbacks */
  sys_ui_register_lvgl_callbacks(&ui_ctx);

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

  /* Update speed and distance */
  if (now - ui_ctx.last_speed_update >= SYS_UI_SPEED_REFRESH_MS)
  {
    ui_ctx.last_speed_update = now;
    ++ui_ctx.frame_counter;
    sys_ui_update_speed_n_distance(&ui_ctx);
  }

  /* Update countdown timer */
  if (now - ui_ctx.last_second_tick >= SYS_UI_COUNTDOWN_MS)
  {
    ui_ctx.last_second_tick = now;
    sys_ui_update_countdown(&ui_ctx);
  }

  /* Update environment telemetry */
  if (now - ui_ctx.last_time_update_screen >= SYS_UI_ENVIRONMENT_MS)
  {
    ui_ctx.last_time_update_screen = now;
    sys_ui_update_env_data(&ui_ctx);
  }

  /* Handle pending main screen redraw */
  if (ui_ctx.view == SYS_UI_VIEW_MAIN && ui_ctx.pending_main_redraw)
  {
    sys_ui_init_full_widget(&ui_ctx);
    sys_ui_register_lvgl_callbacks(&ui_ctx);
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
  w->header_panel         = nullptr;
  w->header_title         = nullptr;
  w->header_subtitle      = nullptr;
  w->status_led           = nullptr;
  w->battery_bar          = nullptr;
  w->speedometer_arc      = nullptr;
  w->speed_label          = nullptr;
  w->speed_unit_label     = nullptr;
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

static void sys_ui_init_data(sys_ui_t *ctx)
{
  ctx->remaining_minutes = 45;
  ctx->remaining_seconds = 0;
  ctx->current_speed     = 0.0f;
  ctx->target_speed      = 12.0f;
  ctx->distance_km       = 0.0f;

  ctx->temperature_C   = 28.0f + static_cast<float>(random(0, 50)) / 10.0f;
  ctx->humidity        = 60.0f + static_cast<float>(random(0, 300)) / 10.0f;
  ctx->air_quality     = 70 + random(0, 30);
  ctx->battery_percent = 80 + random(0, 20);

  const char *seedHistory[SYS_UI_MAX_RENTAL_HISTORY] = { "2026-03-03 09:05 - B-1024", "2026-03-02 17:40 - B-2201",
                                                         "2026-03-01 08:15 - B-3108", "2026-02-29 19:50 - B-1876" };
  ctx->rental_history_count                          = SYS_UI_MAX_RENTAL_HISTORY;
  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    strncpy(ctx->rentalHistory[i], seedHistory[i], sizeof(ctx->rentalHistory[i]) - 1);
    ctx->rentalHistory[i][sizeof(ctx->rentalHistory[i]) - 1] = '\0';
  }
}

static void sys_ui_show_screen(sys_ui_t *ctx, lv_obj_t *screen)
{
  if (ctx == nullptr || screen == nullptr)
    return;

  ctx->widgets.active_screen = screen;
  lv_screen_load(screen);
}

static void sys_ui_main_screen(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  w->main_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(w->main_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(w->main_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Header panel */
  w->header_panel = lv_obj_create(w->main_screen);
  lv_obj_set_pos(w->header_panel, 0, 0);
  lv_obj_set_size(w->header_panel, BSP_DISPLAY_SCREEN_W, 26);
  lv_obj_set_style_bg_color(w->header_panel, lv_color_hex(0x002840), 0);
  lv_obj_set_style_border_width(w->header_panel, 0, 0);
  lv_obj_set_style_radius(w->header_panel, 0, 0);
  lv_obj_set_style_pad_all(w->header_panel, 0, 0);
  lv_obj_remove_flag(w->header_panel, LV_OBJ_FLAG_SCROLLABLE);

  w->header_title = lv_label_create(w->header_panel);
  lv_label_set_text(w->header_title, "SMART BIKE");
  lv_obj_set_pos(w->header_title, 35, 2);
  lv_obj_set_style_text_color(w->header_title, lv_color_hex(SYS_UI_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(w->header_title, &lv_font_montserrat_10, 0);

  w->header_subtitle = lv_label_create(w->header_panel);
  lv_label_set_text(w->header_subtitle, "RENTAL #B-1024");
  lv_obj_set_pos(w->header_subtitle, 35, 13);
  lv_obj_set_style_text_color(w->header_subtitle, lv_color_hex(SYS_UI_COLOR_ACCENT), 0);
  lv_obj_set_style_text_font(w->header_subtitle, &lv_font_montserrat_10, 0);

  w->status_led = lv_obj_create(w->header_panel);
  lv_obj_set_pos(w->status_led, BSP_DISPLAY_SCREEN_W - 55, 9);
  lv_obj_set_size(w->status_led, 8, 8);
  lv_obj_set_style_bg_color(w->status_led, lv_color_hex(SYS_UI_COLOR_SUCCESS), 0);
  lv_obj_set_style_radius(w->status_led, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(w->status_led, 0, 0);

  w->battery_bar = lv_bar_create(w->header_panel);
  lv_obj_set_pos(w->battery_bar, BSP_DISPLAY_SCREEN_W - 30, 8);
  lv_obj_set_size(w->battery_bar, 20, 10);
  lv_bar_set_range(w->battery_bar, 0, 100);
  lv_bar_set_value(w->battery_bar, ctx->battery_percent, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(w->battery_bar, lv_color_hex(0x333333), 0);
  lv_obj_set_style_bg_color(w->battery_bar, lv_color_hex(SYS_UI_COLOR_SUCCESS), LV_PART_INDICATOR);

  /* Speedometer Arc */
  w->speedometer_arc = lv_arc_create(w->main_screen);
  lv_obj_set_pos(w->speedometer_arc, SYS_UI_SPEEDO_CX - SYS_UI_SPEEDO_OUTER_R,
                 SYS_UI_SPEEDO_CY - SYS_UI_SPEEDO_OUTER_R);
  lv_obj_set_size(w->speedometer_arc, SYS_UI_SPEEDO_OUTER_R * 2, SYS_UI_SPEEDO_OUTER_R * 2);
  lv_arc_set_rotation(w->speedometer_arc, 135);
  lv_arc_set_bg_angles(w->speedometer_arc, 0, 270);
  lv_arc_set_range(w->speedometer_arc, 0, 100);
  lv_arc_set_value(w->speedometer_arc, 0);
  lv_obj_set_style_arc_width(w->speedometer_arc, 10, LV_PART_MAIN);
  lv_obj_set_style_arc_width(w->speedometer_arc, 10, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(w->speedometer_arc, lv_color_hex(0x333333), LV_PART_MAIN);
  lv_obj_set_style_arc_color(w->speedometer_arc, lv_color_hex(SYS_UI_COLOR_SUCCESS), LV_PART_INDICATOR);
  lv_obj_remove_style(w->speedometer_arc, nullptr, LV_PART_KNOB);
  lv_obj_remove_flag(w->speedometer_arc, LV_OBJ_FLAG_CLICKABLE);

  w->speed_label = lv_label_create(w->main_screen);
  lv_label_set_text(w->speed_label, "0");
  lv_obj_set_pos(w->speed_label, SYS_UI_SPEEDO_CX - 20, SYS_UI_SPEEDO_CY - 10);
  lv_obj_set_style_text_color(w->speed_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(w->speed_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_align(w->speed_label, LV_TEXT_ALIGN_CENTER, 0);

  w->speed_unit_label = lv_label_create(w->main_screen);
  lv_label_set_text(w->speed_unit_label, "km/h");
  lv_obj_set_pos(w->speed_unit_label, SYS_UI_SPEEDO_CX - 15, SYS_UI_SPEEDO_CY + 20);
  lv_obj_set_style_text_color(w->speed_unit_label, lv_color_hex(SYS_UI_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(w->speed_unit_label, &lv_font_montserrat_10, 0);

  /* Time Card */
  w->time_card = sys_ui_widget_create_card(w->main_screen, SYS_UI_TIME_CARD_X, SYS_UI_TIME_CARD_Y, SYS_UI_TIME_CARD_W,
                                           SYS_UI_TIME_CARD_H, SYS_UI_COLOR_PRIMARY);

  lv_obj_t *time_title = lv_label_create(w->time_card);
  lv_label_set_text(time_title, "REMAIN");
  lv_obj_set_pos(time_title, 10, 2);
  lv_obj_set_style_text_color(time_title, lv_color_hex(SYS_UI_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(time_title, &lv_font_montserrat_10, 0);

  w->time_label = lv_label_create(w->time_card);
  lv_label_set_text(w->time_label, "00:00");
  lv_obj_set_pos(w->time_label, 5, 18);
  lv_obj_set_style_text_color(w->time_label, lv_color_hex(SYS_UI_COLOR_SUCCESS), 0);
  lv_obj_set_style_text_font(w->time_label, &lv_font_montserrat_18, 0);

  w->time_unit_label = lv_label_create(w->time_card);
  lv_label_set_text(w->time_unit_label, "mins");
  lv_obj_set_pos(w->time_unit_label, 25, 38);
  lv_obj_set_style_text_color(w->time_unit_label, lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(w->time_unit_label, &lv_font_montserrat_10, 0);

  /* Distance Card */
  w->distance_card = sys_ui_widget_create_card(w->main_screen, SYS_UI_DIST_CARD_X, SYS_UI_DIST_CARD_Y,
                                               SYS_UI_DIST_CARD_W, SYS_UI_DIST_CARD_H, SYS_UI_COLOR_ACCENT);

  lv_obj_t *dist_title = lv_label_create(w->distance_card);
  lv_label_set_text(dist_title, "DISTANCE");
  lv_obj_set_pos(dist_title, 5, 2);
  lv_obj_set_style_text_color(dist_title, lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(dist_title, &lv_font_montserrat_10, 0);

  w->distance_label = lv_label_create(w->distance_card);
  lv_label_set_text(w->distance_label, "0.00");
  lv_obj_set_pos(w->distance_label, 5, 16);
  lv_obj_set_style_text_color(w->distance_label, lv_color_hex(SYS_UI_COLOR_ACCENT), 0);
  lv_obj_set_style_text_font(w->distance_label, &lv_font_montserrat_16, 0);

  w->distance_unit_label = lv_label_create(w->distance_card);
  lv_label_set_text(w->distance_unit_label, "km");
  lv_obj_set_pos(w->distance_unit_label, 62, 20);
  lv_obj_set_style_text_color(w->distance_unit_label, lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(w->distance_unit_label, &lv_font_montserrat_10, 0);

  /* Environment Card */
  w->env_card = sys_ui_widget_create_card(w->main_screen, SYS_UI_ENV_CARD_X, SYS_UI_ENV_CARD_Y, SYS_UI_ENV_CARD_W,
                                          SYS_UI_ENV_CARD_H, SYS_UI_COLOR_TEXT_DIM);

  lv_obj_t *env_title = lv_label_create(w->env_card);
  lv_label_set_text(env_title, "ENV STATUS");
  lv_obj_set_pos(env_title, 8, 0);
  lv_obj_set_style_text_color(env_title, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(env_title, &lv_font_montserrat_10, 0);

  w->temp_label = lv_label_create(w->env_card);
  lv_label_set_text(w->temp_label, "0.0°C");
  lv_obj_set_pos(w->temp_label, 5, 16);
  lv_obj_set_style_text_color(w->temp_label, lv_color_hex(SYS_UI_COLOR_WARNING), 0);
  lv_obj_set_style_text_font(w->temp_label, &lv_font_montserrat_12, 0);

  w->humidity_label = lv_label_create(w->env_card);
  lv_label_set_text(w->humidity_label, "0%");
  lv_obj_set_pos(w->humidity_label, 5, 32);
  lv_obj_set_style_text_color(w->humidity_label, lv_color_hex(SYS_UI_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(w->humidity_label, &lv_font_montserrat_12, 0);

  w->aqi_label = lv_label_create(w->env_card);
  lv_label_set_text(w->aqi_label, "AQI: --");
  lv_obj_set_pos(w->aqi_label, 5, 48);
  lv_obj_set_style_text_color(w->aqi_label, lv_color_hex(SYS_UI_COLOR_SUCCESS), 0);
  lv_obj_set_style_text_font(w->aqi_label, &lv_font_montserrat_10, 0);

  /* Control Buttons */
  w->settings_btn =
    sys_ui_widget_create_button(w->main_screen, SYS_UI_SETTINGS_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_SETTINGS_BTN_W,
                                SYS_UI_CTRL_BTN_H, "SETTINGS", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);

  w->out_btn = sys_ui_widget_create_button(w->main_screen, SYS_UI_OUT_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_OUT_BTN_W,
                                           SYS_UI_CTRL_BTN_H, "OUT", SYS_UI_COLOR_WARNING, SYS_UI_COLOR_BG);
}

static void sys_ui_createSettingsScreen(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  w->settings_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(w->settings_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(w->settings_screen, LV_OBJ_FLAG_SCROLLABLE);

  w->settings_back_btn =
    sys_ui_widget_create_button(w->settings_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->settings_title = lv_label_create(w->settings_screen);
  lv_label_set_text(w->settings_title, "SETTINGS");
  lv_obj_set_pos(w->settings_title, 90, 15);
  lv_obj_set_style_text_color(w->settings_title, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(w->settings_title, &lv_font_montserrat_18, 0);

  lv_obj_t *bright_label = lv_label_create(w->settings_screen);
  lv_label_set_text(bright_label, "Brightness");
  lv_obj_set_pos(bright_label, 40, 55);
  lv_obj_set_style_text_color(bright_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);

  w->brightness_slider = lv_slider_create(w->settings_screen);
  lv_obj_set_pos(w->brightness_slider, 40, 80);
  lv_obj_set_size(w->brightness_slider, 200, 20);
  lv_slider_set_range(w->brightness_slider, 5, 100);
  lv_slider_set_value(w->brightness_slider, ctx->brightness_percent, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(w->brightness_slider, lv_color_hex(0x333333), LV_PART_MAIN);
  lv_obj_set_style_bg_color(w->brightness_slider, lv_color_hex(SYS_UI_COLOR_PRIMARY), LV_PART_INDICATOR);

  w->brightness_label = lv_label_create(w->settings_screen);
  lv_label_set_text_fmt(w->brightness_label, "%d%%", ctx->brightness_percent);
  lv_obj_set_pos(w->brightness_label, 250, 80);
  lv_obj_set_style_text_color(w->brightness_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);

  lv_obj_t *bg_label = lv_label_create(w->settings_screen);
  lv_label_set_text(bg_label, "Background");
  lv_obj_set_pos(bg_label, 40, 120);
  lv_obj_set_style_text_color(bg_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);

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

static void sys_ui_createOutScreen(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  w->out_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(w->out_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(w->out_screen, LV_OBJ_FLAG_SCROLLABLE);

  w->out_back_btn = sys_ui_widget_create_button(w->out_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->out_title = lv_label_create(w->out_screen);
  lv_label_set_text(w->out_title, "OUT MODE");
  lv_obj_set_pos(w->out_title, 80, 80);
  lv_obj_set_style_text_color(w->out_title, lv_color_hex(SYS_UI_COLOR_WARNING), 0);
  lv_obj_set_style_text_font(w->out_title, &lv_font_montserrat_24, 0);

  lv_obj_t *instr = lv_label_create(w->out_screen);
  lv_label_set_text(instr, "Tap BACK to resume.");
  lv_obj_set_pos(instr, 80, 130);
  lv_obj_set_style_text_color(instr, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
}

static void sys_ui_createTimeHistoryScreen(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  w->time_history_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(w->time_history_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(w->time_history_screen, LV_OBJ_FLAG_SCROLLABLE);

  w->time_back_btn =
    sys_ui_widget_create_button(w->time_history_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->time_history_title = lv_label_create(w->time_history_screen);
  lv_label_set_text(w->time_history_title, "RENTAL HISTORY");
  lv_obj_set_pos(w->time_history_title, 60, 15);
  lv_obj_set_style_text_color(w->time_history_title, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(w->time_history_title, &lv_font_montserrat_18, 0);

  for (int i = 0; i < 4; ++i)
  {
    w->history_labels[i] = lv_label_create(w->time_history_screen);
    lv_label_set_text(w->history_labels[i], "");
    lv_obj_set_pos(w->history_labels[i], 40, 55 + i * 20);
    lv_obj_set_style_text_color(w->history_labels[i], lv_color_hex(SYS_UI_COLOR_TEXT), 0);
    lv_obj_set_style_text_font(w->history_labels[i], &lv_font_montserrat_12, 0);
  }

  w->time_remaining_label = lv_label_create(w->time_history_screen);
  lv_label_set_text(w->time_remaining_label, "Remaining: 00:00");
  lv_obj_set_pos(w->time_remaining_label, 40, 140);
  lv_obj_set_style_text_color(w->time_remaining_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);

  w->extend_btn =
    sys_ui_widget_create_button(w->time_history_screen, SYS_UI_EXTEND_BTN_X, SYS_UI_EXTEND_BTN_Y, SYS_UI_EXTEND_BTN_W,
                                SYS_UI_EXTEND_BTN_H, "+30 min", SYS_UI_COLOR_SUCCESS, SYS_UI_COLOR_BG);
}

static void sys_ui_createDistanceScreen(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  w->distance_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(w->distance_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(w->distance_screen, LV_OBJ_FLAG_SCROLLABLE);

  w->distance_back_btn =
    sys_ui_widget_create_button(w->distance_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->distance_title = lv_label_create(w->distance_screen);
  lv_label_set_text(w->distance_title, "DISTANCE STATS");
  lv_obj_set_pos(w->distance_title, 70, 15);
  lv_obj_set_style_text_color(w->distance_title, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(w->distance_title, &lv_font_montserrat_18, 0);

  w->total_distance_label = lv_label_create(w->distance_screen);
  lv_label_set_text(w->total_distance_label, "Total: 0.00 km");
  lv_obj_set_pos(w->total_distance_label, 30, 50);
  lv_obj_set_style_text_color(w->total_distance_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);

  w->avg_speed_label = lv_label_create(w->distance_screen);
  lv_label_set_text(w->avg_speed_label, "Avg speed: 0.0 km/h");
  lv_obj_set_pos(w->avg_speed_label, 30, 70);
  lv_obj_set_style_text_color(w->avg_speed_label, lv_color_hex(SYS_UI_COLOR_TEXT), 0);

  w->distance_chart = lv_chart_create(w->distance_screen);
  lv_obj_set_pos(w->distance_chart, 30, 95);
  lv_obj_set_size(w->distance_chart, 260, 120);
  lv_chart_set_type(w->distance_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(w->distance_chart, 16);
  lv_obj_set_style_bg_color(w->distance_chart, lv_color_hex(0x111111), 0);

  w->distance_series =
    lv_chart_add_series(w->distance_chart, lv_color_hex(SYS_UI_COLOR_ACCENT), LV_CHART_AXIS_PRIMARY_Y);
}

static void sys_ui_createTemperatureScreen(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  w->temp_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(w->temp_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(w->temp_screen, LV_OBJ_FLAG_SCROLLABLE);

  w->temp_back_btn =
    sys_ui_widget_create_button(w->temp_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  w->temp_title = lv_label_create(w->temp_screen);
  lv_label_set_text(w->temp_title, "TEMPERATURE");
  lv_obj_set_pos(w->temp_title, 90, 15);
  lv_obj_set_style_text_color(w->temp_title, lv_color_hex(SYS_UI_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(w->temp_title, &lv_font_montserrat_18, 0);

  w->temp_chart = lv_chart_create(w->temp_screen);
  lv_obj_set_pos(w->temp_chart, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_GRAPH_Y);
  lv_obj_set_size(w->temp_chart, SYS_UI_TEMP_GRAPH_W, SYS_UI_TEMP_GRAPH_H);
  lv_chart_set_type(w->temp_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(w->temp_chart, 60);
  lv_obj_set_style_bg_color(w->temp_chart, lv_color_hex(0x111111), 0);

  w->temp_series = lv_chart_add_series(w->temp_chart, lv_color_hex(SYS_UI_COLOR_ACCENT), LV_CHART_AXIS_PRIMARY_Y);

  w->temp_range_label = lv_label_create(w->temp_screen);
  lv_label_set_text(w->temp_range_label, "Collecting...");
  lv_obj_set_pos(w->temp_range_label, SYS_UI_TEMP_GRAPH_X, SYS_UI_TEMP_GRAPH_Y - 15);
  lv_obj_set_style_text_color(w->temp_range_label, lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(w->temp_range_label, &lv_font_montserrat_10, 0);

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

static void sys_ui_init_full_widget(sys_ui_t *ctx)
{
  if (ctx->widgets.main_screen == nullptr)
  {
    sys_ui_main_screen(ctx);
  }

  sys_ui_show_screen(ctx, ctx->widgets.main_screen);
  sys_ui_update_time(ctx, ctx->remaining_minutes, ctx->remaining_seconds);
  sys_ui_update_distance(ctx, ctx->distance_km);
  sys_ui_update_env(ctx, ctx->temperature_C, ctx->humidity, ctx->air_quality);
  sys_ui_update_speed(ctx, static_cast<int>(ctx->current_speed));
}

static void sys_ui_update_speed(sys_ui_t *ctx, int speedKph)
{
  speedKph = SYS_UI_CLAMP(speedKph, 0, 40);

  if (ctx->prev_speed_int == speedKph)
    return;
  ctx->prev_speed_int = speedKph;

  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->speedometer_arc != nullptr)
  {
    int arcValue = (speedKph * 100) / 40;
    lv_arc_set_value(w->speedometer_arc, arcValue);

    lv_color_t arcColor;
    if (speedKph <= 20)
      arcColor = lv_color_hex(SYS_UI_COLOR_SUCCESS);
    else if (speedKph <= 30)
      arcColor = lv_color_hex(SYS_UI_COLOR_WARNING);
    else
      arcColor = lv_color_hex(SYS_UI_COLOR_DANGER);
    lv_obj_set_style_arc_color(w->speedometer_arc, arcColor, LV_PART_INDICATOR);
  }

  if (w->speed_label != nullptr)
  {
    lv_label_set_text_fmt(w->speed_label, "%d", speedKph);
  }
}

static void sys_ui_update_time(sys_ui_t *ctx, int minutes, int seconds)
{
  sys_ui_widgets_t *w = &ctx->widgets;
  if (w->time_label == nullptr)
    return;

  lv_label_set_text_fmt(w->time_label, "%02d:%02d", minutes, seconds);

  lv_color_t timeColor;
  if (minutes < 5)
    timeColor = lv_color_hex(SYS_UI_COLOR_DANGER);
  else if (minutes < 10)
    timeColor = lv_color_hex(SYS_UI_COLOR_WARNING);
  else
    timeColor = lv_color_hex(SYS_UI_COLOR_SUCCESS);
  lv_obj_set_style_text_color(w->time_label, timeColor, 0);
}

static void sys_ui_update_distance(sys_ui_t *ctx, float distance_km)
{
  sys_ui_widgets_t *w = &ctx->widgets;
  if (w->distance_label == nullptr)
    return;

  lv_label_set_text_fmt(w->distance_label, "%.2f", distance_km);
}

static void sys_ui_update_env(sys_ui_t *ctx, float temperature_C, float humidity, int air_quality)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->temp_label != nullptr)
  {
    lv_label_set_text_fmt(w->temp_label, "%.1f°C", temperature_C);
  }

  if (w->humidity_label != nullptr)
  {
    lv_label_set_text_fmt(w->humidity_label, "%.0f%%", humidity);
  }

  if (w->aqi_label != nullptr)
  {
    const char *status = (air_quality >= 80) ? "Good" : ((air_quality >= 50) ? "Fair" : "Poor");
    lv_label_set_text_fmt(w->aqi_label, "AQI:%d %s", air_quality, status);

    lv_color_t aqiColor;
    if (air_quality >= 80)
      aqiColor = lv_color_hex(SYS_UI_COLOR_SUCCESS);
    else if (air_quality >= 50)
      aqiColor = lv_color_hex(SYS_UI_COLOR_WARNING);
    else
      aqiColor = lv_color_hex(SYS_UI_COLOR_DANGER);
    lv_obj_set_style_text_color(w->aqi_label, aqiColor, 0);
  }
}

static void sys_ui_update_speed_n_distance(sys_ui_t *ctx)
{
  float speedDelta  = static_cast<float>(random(-25, 26)) / 10.0f;
  ctx->target_speed = SYS_UI_CLAMP(ctx->target_speed + speedDelta, 0.0f, 35.0f);

  ctx->current_speed += (ctx->target_speed - ctx->current_speed) * 0.2f;
  ctx->distance_km += ctx->current_speed * (SYS_UI_SPEED_REFRESH_MS / 1000.0f) / 3600.0f;

  if (ctx->view == SYS_UI_VIEW_MAIN)
  {
    sys_ui_update_speed(ctx, static_cast<int>(ctx->current_speed));
    if (ctx->frame_counter % 10 == 0)
    {
      sys_ui_update_distance(ctx, ctx->distance_km);
    }
  }
  else
  {
    ctx->pending_main_redraw = true;
  }

  if (ctx->frame_counter % 60 == 0)
  {
    sys_ui_logDistanceSample(ctx, ctx->distance_km);
  }

  if (ctx->view == SYS_UI_VIEW_DISTANCE)
  {
    sys_ui_drawDistanceOverlay(ctx);
  }
}

static void sys_ui_update_countdown(sys_ui_t *ctx)
{
  if (ctx->remaining_seconds > 0)
  {
    --ctx->remaining_seconds;
  }
  else if (ctx->remaining_minutes > 0)
  {
    --ctx->remaining_minutes;
    ctx->remaining_seconds = 59;
  }

  if (ctx->view == SYS_UI_VIEW_MAIN)
  {
    sys_ui_update_time(ctx, ctx->remaining_minutes, ctx->remaining_seconds);
  }
  else if (ctx->view == SYS_UI_VIEW_TIME)
  {
    sys_ui_drawTimeHistoryOverlay(ctx);
  }
  else
  {
    ctx->pending_main_redraw = true;
  }
}

static void sys_ui_update_env_data(sys_ui_t *ctx)
{
  ctx->temperature_C = SYS_UI_CLAMP(ctx->temperature_C + static_cast<float>(random(-10, 11)) / 10.0f, 20.0f, 40.0f);
  ctx->humidity      = SYS_UI_CLAMP(ctx->humidity + static_cast<float>(random(-20, 21)) / 10.0f, 40.0f, 95.0f);
  ctx->air_quality   = SYS_UI_CLAMP(ctx->air_quality + random(-5, 6), 30, 100);

  sys_ui_logTemperatureSample(ctx, ctx->temperature_C, OS_GET_TICK());

  if (ctx->view == SYS_UI_VIEW_MAIN)
  {
    sys_ui_update_env(ctx, ctx->temperature_C, ctx->humidity, ctx->air_quality);
  }
  else if (ctx->view == SYS_UI_VIEW_TEMPERATURE)
  {
    sys_ui_refreshTemperatureOverlay(ctx);
  }
  else
  {
    ctx->pending_main_redraw = true;
  }
}

static void sys_ui_drawSettingsOverlay(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->settings_screen == nullptr)
  {
    sys_ui_createSettingsScreen(ctx);
  }

  if (w->brightness_slider != nullptr)
  {
    lv_slider_set_value(w->brightness_slider, ctx->brightness_percent, LV_ANIM_OFF);
  }
  if (w->brightness_label != nullptr)
  {
    lv_label_set_text_fmt(w->brightness_label, "%d%%", ctx->brightness_percent);
  }

  sys_ui_show_screen(ctx, w->settings_screen);
}

static void sys_ui_drawOutOverlay(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->out_screen == nullptr)
  {
    sys_ui_createOutScreen(ctx);
  }

  sys_ui_show_screen(ctx, w->out_screen);
}

static void sys_ui_drawTimeHistoryOverlay(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->time_history_screen == nullptr)
  {
    sys_ui_createTimeHistoryScreen(ctx);
  }

  for (int i = 0; i < 4; ++i)
  {
    if (w->history_labels[i] != nullptr)
    {
      if (i < ctx->rental_history_count)
      {
        lv_label_set_text(w->history_labels[i], ctx->rentalHistory[i]);
      }
      else
      {
        lv_label_set_text(w->history_labels[i], "");
      }
    }
  }

  if (w->time_remaining_label != nullptr)
  {
    lv_label_set_text_fmt(w->time_remaining_label, "Remaining: %02d:%02d", ctx->remaining_minutes,
                          ctx->remaining_seconds);
  }

  sys_ui_show_screen(ctx, w->time_history_screen);
}

static void sys_ui_drawDistanceOverlay(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->distance_screen == nullptr)
  {
    sys_ui_createDistanceScreen(ctx);
  }

  size_t now   = OS_GET_TICK();
  float  hours = (now - ctx->session_start_ms) / 3600000.0f;
  if (hours < 0.001f)
    hours = 0.001f;
  float avgSpeed = ctx->distance_km / hours;

  if (w->total_distance_label != nullptr)
  {
    lv_label_set_text_fmt(w->total_distance_label, "Total: %.2f km", ctx->distance_km);
  }

  if (w->avg_speed_label != nullptr)
  {
    lv_label_set_text_fmt(w->avg_speed_label, "Avg speed: %.1f km/h", avgSpeed);
  }

  if (w->distance_chart != nullptr && w->distance_series != nullptr)
  {
    int count = ctx->distance_history_count;
    lv_chart_set_point_count(w->distance_chart, count > 0 ? count : 1);
    for (int i = 0; i < count && i < 16; ++i)
    {
      lv_chart_set_value_by_id(w->distance_chart, w->distance_series, i, (int) (ctx->distanceHistory[i] * 100));
    }
    lv_chart_refresh(w->distance_chart);
  }

  sys_ui_show_screen(ctx, w->distance_screen);
}

static void sys_ui_drawTemperatureOverlay(sys_ui_t *ctx)
{
  sys_ui_widgets_t *w = &ctx->widgets;

  if (w->temp_screen == nullptr)
  {
    sys_ui_createTemperatureScreen(ctx);
  }

  int count = ctx->temperature_sample_count;
  if (w->temp_chart != nullptr && w->temp_series != nullptr && count > 1)
  {
    int zoom      = ctx->temperature_zoom;
    int panOffset = ctx->temperature_pan;
    int window    = count / zoom;
    if (window < 2)
      window = 2;
    if (window > count)
      window = count;
    if (panOffset < 0)
      panOffset = 0;
    if (panOffset > count - window)
      panOffset = count - window;

    float minTemp = ctx->temperatureHistory[panOffset];
    float maxTemp = ctx->temperatureHistory[panOffset];
    for (int i = 0; i < window; ++i)
    {
      float v = ctx->temperatureHistory[panOffset + i];
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
        span = (ctx->temperatureTimestamps[panOffset + window - 1] - ctx->temperatureTimestamps[panOffset]) / 1000.0f;
      }
      lv_label_set_text_fmt(w->temp_range_label, "%.1f-%.1fC (%.1fs)", minTemp, maxTemp, span);
    }

    lv_chart_set_point_count(w->temp_chart, window);
    lv_chart_set_range(w->temp_chart, LV_CHART_AXIS_PRIMARY_Y, (int) (minTemp * 10) - 5, (int) (maxTemp * 10) + 5);

    for (int i = 0; i < window; ++i)
    {
      lv_chart_set_value_by_id(w->temp_chart, w->temp_series, i, (int) (ctx->temperatureHistory[panOffset + i] * 10));
    }
    lv_chart_refresh(w->temp_chart);
  }

  sys_ui_show_screen(ctx, w->temp_screen);
}

static void sys_ui_enterView(sys_ui_t *ctx, sys_ui_view_t view)
{
  ctx->view = view;
  switch (view)
  {
  case SYS_UI_VIEW_SETTINGS:
    sys_ui_drawSettingsOverlay(ctx);
    if (ctx->widgets.settings_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.settings_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->widgets.brightness_slider != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.brightness_slider, sys_ui_brightness_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    }
    for (int i = 0; i < 3; ++i)
    {
      if (ctx->widgets.color_btns[i] != nullptr)
      {
        lv_obj_add_event_cb(ctx->widgets.color_btns[i], sys_ui_color_btn_cb, LV_EVENT_CLICKED, (void *) (intptr_t) i);
      }
    }
    break;
  case SYS_UI_VIEW_OUT:
    sys_ui_drawOutOverlay(ctx);
    if (ctx->widgets.out_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.out_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  case SYS_UI_VIEW_TIME:
    sys_ui_drawTimeHistoryOverlay(ctx);
    if (ctx->widgets.time_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.time_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->widgets.extend_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.extend_btn, sys_ui_extend_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  case SYS_UI_VIEW_DISTANCE:
    sys_ui_drawDistanceOverlay(ctx);
    if (ctx->widgets.distance_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.distance_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  case SYS_UI_VIEW_TEMPERATURE:
    sys_ui_drawTemperatureOverlay(ctx);
    if (ctx->widgets.temp_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.temp_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->widgets.zoom_minus_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.zoom_minus_btn, sys_ui_zoom_minus_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->widgets.zoom_plus_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.zoom_plus_btn, sys_ui_zoom_plus_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->widgets.pan_left_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.pan_left_btn, sys_ui_pan_left_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->widgets.pan_right_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->widgets.pan_right_btn, sys_ui_pan_right_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  default: break;
  }
}

static void sys_ui_returnToMain(sys_ui_t *ctx)
{
  ctx->view                = SYS_UI_VIEW_MAIN;
  ctx->pending_main_redraw = false;
  sys_ui_init_full_widget(ctx);
  sys_ui_register_lvgl_callbacks(ctx);
}

static void sys_ui_logDistanceSample(sys_ui_t *ctx, float value)
{
  if (ctx->distance_history_count < SYS_UI_MAX_DISTANCE_LOG)
  {
    ctx->distanceHistory[ctx->distance_history_count++] = value;
    return;
  }

  memmove(&ctx->distanceHistory[0], &ctx->distanceHistory[1],
          (SYS_UI_MAX_DISTANCE_LOG - 1) * sizeof(ctx->distanceHistory[0]));
  ctx->distanceHistory[SYS_UI_MAX_DISTANCE_LOG - 1] = value;
}

static void sys_ui_logTemperatureSample(sys_ui_t *ctx, float value, size_t timestamp)
{
  if (ctx->temperature_sample_count < SYS_UI_MAX_TEMP_SAMPLES)
  {
    ctx->temperatureHistory[ctx->temperature_sample_count]    = value;
    ctx->temperatureTimestamps[ctx->temperature_sample_count] = timestamp;
    ++ctx->temperature_sample_count;
    return;
  }

  memmove(&ctx->temperatureHistory[0], &ctx->temperatureHistory[1],
          (SYS_UI_MAX_TEMP_SAMPLES - 1) * sizeof(ctx->temperatureHistory[0]));
  memmove(&ctx->temperatureTimestamps[0], &ctx->temperatureTimestamps[1],
          (SYS_UI_MAX_TEMP_SAMPLES - 1) * sizeof(ctx->temperatureTimestamps[0]));
  ctx->temperatureHistory[SYS_UI_MAX_TEMP_SAMPLES - 1]    = value;
  ctx->temperatureTimestamps[SYS_UI_MAX_TEMP_SAMPLES - 1] = timestamp;
}

static void sys_ui_refreshTemperatureOverlay(sys_ui_t *ctx)
{
  int max_pan = (ctx->temperature_sample_count > 2) ? (ctx->temperature_sample_count - 2) : 0;
  if (ctx->temperature_pan > max_pan)
    ctx->temperature_pan = max_pan;
  if (ctx->temperature_pan < 0)
    ctx->temperature_pan = 0;

  sys_ui_drawTemperatureOverlay(ctx);
}

/* ============== LVGL Callbacks ============== */
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  sys_ui_t *ctx = (sys_ui_t *) lv_indev_get_user_data(indev);
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
  sys_ui_enterView(&ui_ctx, SYS_UI_VIEW_SETTINGS);
}

static void sys_ui_out_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Out button pressed\n");
  sys_ui_enterView(&ui_ctx, SYS_UI_VIEW_OUT);
}

static void sys_ui_back_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Back button pressed\n");
  sys_ui_returnToMain(&ui_ctx);
}

static void sys_ui_extend_btn_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Extend button pressed\n");
  ui_ctx.remaining_minutes += 30;
  sys_ui_drawTimeHistoryOverlay(&ui_ctx);
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
    sys_ui_refreshTemperatureOverlay(&ui_ctx);
    LOG_DBG("[LVGL] Zoom- pressed, zoom=%d\n", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_zoom_plus_btn_cb(lv_event_t *e)
{
  (void) e;
  if (ui_ctx.temperature_zoom < 4)
  {
    ++ui_ctx.temperature_zoom;
    sys_ui_refreshTemperatureOverlay(&ui_ctx);
    LOG_DBG("[LVGL] Zoom+ pressed, zoom=%d\n", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_pan_left_btn_cb(lv_event_t *e)
{
  (void) e;
  ui_ctx.temperature_pan -= 5;
  sys_ui_refreshTemperatureOverlay(&ui_ctx);
  LOG_DBG("[LVGL] Pan left pressed, pan=%d\n", ui_ctx.temperature_pan);
}

static void sys_ui_pan_right_btn_cb(lv_event_t *e)
{
  (void) e;
  ui_ctx.temperature_pan += 5;
  sys_ui_refreshTemperatureOverlay(&ui_ctx);
  LOG_DBG("[LVGL] Pan right pressed, pan=%d\n", ui_ctx.temperature_pan);
}

static void sys_ui_time_card_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Time card pressed\n");
  sys_ui_enterView(&ui_ctx, SYS_UI_VIEW_TIME);
}

static void sys_ui_distance_card_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Distance card pressed\n");
  sys_ui_enterView(&ui_ctx, SYS_UI_VIEW_DISTANCE);
}

static void sys_ui_env_card_cb(lv_event_t *e)
{
  (void) e;
  LOG_DBG("[LVGL] Environment card pressed\n");
  sys_ui_enterView(&ui_ctx, SYS_UI_VIEW_TEMPERATURE);
}

static void sys_ui_register_lvgl_callbacks(sys_ui_t *ctx)
{
  if (ctx == nullptr)
    return;

  sys_ui_widgets_t *w = &ctx->widgets;

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
    lv_obj_add_flag(w->time_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(w->time_card, sys_ui_time_card_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (w->distance_card != nullptr)
  {
    lv_obj_add_flag(w->distance_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(w->distance_card, sys_ui_distance_card_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (w->env_card != nullptr)
  {
    lv_obj_add_flag(w->env_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(w->env_card, sys_ui_env_card_cb, LV_EVENT_CLICKED, nullptr);
  }
}

/* End of file -------------------------------------------------------- */
