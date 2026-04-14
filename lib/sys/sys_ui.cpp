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

#include "bsp_dust_sensor.h"
#include "bsp_led.h"
#include "bsp_sdcard.h"
#include "common_type.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_input.h"
#include "sys_manager.h"

#include <math.h>
#include <string.h>

#ifndef PI
#define PI (3.14159265358979323846)
#endif

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_ui, LOG_LEVEL_DBG);

#define SYS_UI_LED_RGB_TASK_MS          (100)
#define SYS_UI_LED_DEFAULT_BRIGHTNESS   (80)

// Platform UI settings
#define SYS_UI_COLOR_BG                 SYS_UI_WIDGET_COLOR_DARK_NAVY
#define SYS_UI_COLOR_BG_CARD            SYS_UI_WIDGET_COLOR_BLUE_GRAY
#define SYS_UI_COLOR_PRIMARY            SYS_UI_WIDGET_COLOR_CYAN
#define SYS_UI_COLOR_ACCENT             SYS_UI_WIDGET_COLOR_GOLD
#define SYS_UI_COLOR_SUCCESS            SYS_UI_WIDGET_COLOR_GREEN
#define SYS_UI_COLOR_WARNING            SYS_UI_WIDGET_COLOR_ORANGE
#define SYS_UI_COLOR_DANGER             SYS_UI_WIDGET_COLOR_RED
#define SYS_UI_COLOR_TEXT               SYS_UI_WIDGET_COLOR_WHITE
#define SYS_UI_COLOR_TEXT_DIM           SYS_UI_WIDGET_COLOR_GRAY
#define SYS_UI_DATA_HISTORY_SAMPLES     (100)
#define SYS_UI_BRIGHTNESS_PERCENT_OFF   (0)

// Timing
#define SYS_UI_COUNTDOWN_MS             (1000)
#define SYS_UI_ENVIRONMENT_MS           (3000)

// Data limits
#define SYS_UI_MAX_RENTAL_HISTORY       (4)
#define SYS_UI_MAX_DISTANCE_LOG         (16)
#define SYS_UI_MAX_TEMP_SAMPLES         (120)

// Top control buttons
#define SYS_UI_CTRL_BTN_Y               (5)
#define SYS_UI_CTRL_BTN_H               (25)
#define SYS_UI_SETTINGS_BTN_X           (10)
#define SYS_UI_SETTINGS_BTN_W           (90)
#define SYS_UI_OUT_BTN_X                (220)
#define SYS_UI_OUT_BTN_W                (90)

// Speedometer arc
#define SYS_UI_SPEEDO_CX                (107)
#define SYS_UI_SPEEDO_CY                (106)
#define SYS_UI_SPEEDO_OUTER_R           (74)
#define SYS_UI_SPEEDO_INNER_R           (54)

// Compass panel
#define SYS_UI_MAP_PANEL_X              (2)
#define SYS_UI_MAP_PANEL_Y              (183)
#define SYS_UI_MAP_PANEL_W              (50)
#define SYS_UI_MAP_PANEL_H              (52)
#define SYS_UI_COMPASS_CX               (22)
#define SYS_UI_COMPASS_CY               (210)
#define SYS_UI_COMPASS_R                (15)
#define SYS_UI_HEADING_TEXT_X           (SYS_UI_COMPASS_CX - 12)
#define SYS_UI_HEADING_TEXT_Y           (SYS_UI_COMPASS_CY - 12)
#define SYS_UI_COMPASS_DEG_X            (SYS_UI_COMPASS_CX - 12)
#define SYS_UI_COMPASS_DEG_Y            (SYS_UI_COMPASS_CY + 5)
#define SYS_UI_COMPASS_PANEL_BG_COLOR   (0x0A1520)

// Right panel info cards
#define SYS_UI_CARD_X                   (218)
#define SYS_UI_CARD_W                   (100)
#define SYS_UI_TIME_CARD_Y              (34)
#define SYS_UI_TIME_CARD_H              (52)
#define SYS_UI_DIST_CARD_Y              (91)
#define SYS_UI_DIST_CARD_H              (40)
#define SYS_UI_ENV_CARD_Y               (136)
#define SYS_UI_ENV_CARD_H               (100)

// Shared sub-screen back button
#define SYS_UI_BACK_BTN_X               (10)
#define SYS_UI_BACK_BTN_Y               (10)
#define SYS_UI_BACK_BTN_W               (60)
#define SYS_UI_BACK_BTN_H               (25)
#define SYS_UI_BACK_BTN_LABEL           "BACK"
#define SYS_UI_BACK_BTN_COLOR           SYS_UI_COLOR_ACCENT
#define SYS_UI_BACK_BTN_TEXT_COLOR      SYS_UI_COLOR_TEXT

// Screen out
#define SYS_UI_CONFIRM_BTN_X            (75)
#define SYS_UI_CONFIRM_BTN_Y            (105)
#define SYS_UI_CONFIRM_BTN_W            (120)
#define SYS_UI_CONFIRM_BTN_H            (50)
#define SYS_UI_CONFIRM_BTN_LABEL        "CONFIRM"
#define SYS_UI_CONFIRM_BTN_COLOR        SYS_UI_COLOR_SUCCESS
#define SYS_UI_CONFIRM_BTN_TEXT_COLOR   SYS_UI_COLOR_TEXT
#define SYS_UI_OUT_LABEL_X              (50)
#define SYS_UI_OUT_LABEL_Y              (81)
#define SYS_UI_OUT_LABEL_TEXT           "Do you want to quit the bike?"

// Settings screen
#define SYS_UI_SWATCH_ROW_Y             (130)
#define SYS_UI_SWATCH_SIZE              (36)
#define SYS_UI_SWATCH_SPAN              (50)

// Distance / Temperature sub-screens
#define SYS_UI_EXTEND_BTN_X             (180)
#define SYS_UI_EXTEND_BTN_Y             (180)
#define SYS_UI_EXTEND_BTN_W             (120)
#define SYS_UI_EXTEND_BTN_H             (30)
#define SYS_UI_TEMP_GRAPH_X             (20)
#define SYS_UI_TEMP_GRAPH_Y             (50)
#define SYS_UI_TEMP_GRAPH_W             (280)
#define SYS_UI_TEMP_GRAPH_H             (140)
#define SYS_UI_TEMP_BTN_Y               (200)
#define SYS_UI_TEMP_BTN_W               (60)
#define SYS_UI_TEMP_BTN_H               (24)
#define SYS_UI_TEMP_BTN_GAP             (10)

// Fusion screen
#define SYS_UI_FUSION_LABEL_X           (72)
#define SYS_UI_FUSION_LABEL_Y           (15)
#define SYS_UI_FUSION_LABEL_TEXT        "FUSION STATS"
#define SYS_UI_FUSION_MAX_SPEED_TEXT    "Max Speed: %.1f km/h"
#define SYS_UI_FUSION_MAX_SPEED_X       (30)
#define SYS_UI_FUSION_MAX_SPEED_Y       (50)
#define SYS_UI_FUSION_AVG_SPEED_TEXT    "Avg Speed: %.1f km/h"
#define SYS_UI_FUSION_AVG_SPEED_X       (30)
#define SYS_UI_FUSION_AVG_SPEED_Y       (70)
#define SYS_UI_FUSION_CHART_X           (30)
#define SYS_UI_FUSION_CHART_Y           (95)
#define SYS_UI_FUSION_CHART_W           (260)
#define SYS_UI_FUSION_CHART_H           (120)
#define SYS_UI_FUSION_CHART_POINTS      (16)
#define SYS_UI_FUSION_CHART_COLOR       BSP_DISPLAY_RGB_TO_HEX(17, 17, 17)  // Dark gray
#define SYS_UI_FUSION_CHART_POINT_COLOR SYS_UI_WIDGET_COLOR_GOLD

// Lock screen
#define SYS_UI_QR_PATH                  "/img/qr.bin"
#define SYS_UI_QR_LABEL                 "SCAN TO UNLOCK"
#define SYS_UI_QR_LABEL_X               (80)
#define SYS_UI_QR_LABEL_Y               (20)
#define SYS_UI_QR_LABEL_FAIL            "[QR]"
#define SYS_UI_QR_LABEL_FAIL_X          (100)
#define SYS_UI_QR_LABEL_FAIL_Y          (80)
#define SYS_UI_QR_LABEL_FAIL_FONT       (&lv_font_montserrat_28)
#define SYS_UI_QR_LABEL_FONT            (&lv_font_montserrat_18)
#define SYS_UI_QR_WIDTH                 (160)
#define SYS_UI_QR_HEIGHT                (160)
#define SYS_UI_QR_X                     (80)
#define SYS_UI_QR_Y                     (50)
#define SYS_UI_DEVICE_ID_LABEL_X        (80)
#define SYS_UI_DEVICE_ID_LABEL_Y        (215)
#define SYS_UI_DEVICE_ID_LABEL_FONT     (&lv_font_montserrat_10)

/* Background color palette for settings screen
 * INFO(index, R, G, B, label)            */
#define SYS_UI_BG_COLOR_TABLE(INFO) \
  INFO(0, 13, 27, 42, "Navy")       \
  INFO(1, 3, 4, 94, "Ocean")        \
  INFO(2, 10, 10, 10, "Black")
#define SYS_UI_BG_COLOR_COUNT         (3)
#define SYS_UI_CLAMP(val, minv, maxv) ((val) < (minv) ? (minv) : ((val) > (maxv) ? (maxv) : (val)))

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  lv_obj_t *active_screen;
  // Main screen
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
  // Settings screen
  lv_obj_t *settings_screen;
  lv_obj_t *settings_title;
  lv_obj_t *settings_back_btn;
  lv_obj_t *brightness_slider;
  lv_obj_t *brightness_label;
  lv_obj_t *color_btns[SYS_UI_BG_COLOR_COUNT];
  // Out screen
  lv_obj_t *out_screen;
  lv_obj_t *out_back_btn;
  lv_obj_t *out_confirm_btn;
  // Lock screen
  lv_obj_t      *lock_screen;
  lv_obj_t      *lock_qr_img;
  lv_obj_t      *device_id_label;
  lv_image_dsc_t lock_qr_dsc;
  // Time history screen
  lv_obj_t *time_history_screen;
  lv_obj_t *time_history_title;
  lv_obj_t *time_back_btn;
  lv_obj_t *time_remaining_label;
  lv_obj_t *history_labels[SYS_UI_MAX_RENTAL_HISTORY];
  lv_obj_t *extend_btn;
  // Fusion screen
  lv_obj_t          *fusion_screen;
  lv_obj_t          *distance_title;
  lv_obj_t          *distance_back_btn;
  lv_obj_t          *max_speed_lable;
  lv_obj_t          *avg_speed_label;
  lv_obj_t          *distance_chart;
  lv_chart_series_t *distance_series;
  // Temperature screen
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
  // Timing
  size_t last_speed_update;
  size_t last_second_tick;
  size_t last_time_update_screen;
  size_t session_start_ms;
  size_t frame_counter;
  // Motion data
  float             current_speed;
  float             max_speed;
  float             target_speed;
  float             distance_km;
  int               prev_speed_int;
  float             compass_heading_deg;
  sys_fusion_data_t fusion;
  // Rental time
  int remaining_minutes;
  int remaining_seconds;
  // Environment
  float    temperature_C;
  float    humidity;
  uint16_t air_quality;
  // Device state
  int    battery_percent;
  int    brightness_percent;
  size_t background_color;
  // Navigation
  sys_ui_view_t view;
  sys_ui_view_t last_view;
  bool          pending_main_redraw;
  uint16_t      last_touch_x;
  uint16_t      last_touch_y;
  // History logs
  char   rental_history[SYS_UI_MAX_RENTAL_HISTORY][32];
  int    rental_history_count;
  float  distance_history[SYS_UI_MAX_DISTANCE_LOG];
  int    distance_history_count;
  float  temp_history[SYS_UI_MAX_TEMP_SAMPLES];
  size_t temp_timestamps[SYS_UI_MAX_TEMP_SAMPLES];
  int    temperature_sample_count;
  int    temperature_zoom;
  int    temperature_graph_offset;
} sys_ui_context_t;

/* Public variables --------------------------------------------------- */
sys_ui_data_status_t g_sys_ui_data_status = { 0 };

/* Private variables -------------------------------------------------- */
static sys_ui_context_t ui_ctx;

static float  distance_history_data[SYS_UI_DATA_HISTORY_SAMPLES];
static int    distance_history_count = 0;
static float  temperature_history_data[SYS_UI_DATA_HISTORY_SAMPLES];
static size_t temperature_time_data[SYS_UI_DATA_HISTORY_SAMPLES];
static int    temperature_history_count = 0;

OS_SEM_DEFINE_STATIC(sys_ui_wakeup_sem);

/* Private function prototypes ---------------------------------------- */
// Core
static void              sys_ui_process_active(void);
static void              sys_ui_process_idle(void);
static void              sys_ui_process_locked(void);
static void              sys_ui_show_screen(lv_obj_t *screen);
static void              sys_ui_change_screen(sys_ui_view_t view);
static void              sys_ui_init_data(void);
static void              sys_ui_init_history_data(void);
static void              sys_ui_init_all_widgets(void);
static void              sys_ui_reset_all_widgets(sys_ui_widgets_t *screen);
static void              sys_ui_register_callbacks(void);
static status_function_t sys_ui_image_draw(lv_obj_t     *parent,
                                           const char   *path,
                                           uint16_t      width,
                                           uint16_t      height,
                                           int16_t       x,
                                           int16_t       y,
                                           lv_img_dsc_t *out_dsc,
                                           lv_obj_t    **out_obj);
// Main screen
static void sys_ui_main_screen_create(void);
static void sys_ui_main_screen_update_speed(int speed_kph);
static void sys_ui_main_screen_update_time(int minutes, int seconds);
static void sys_ui_main_screen_update_distance(float distance_km);
static void sys_ui_main_screen_update_env(float temperature_C, float humidity, int air_quality);
static void sys_ui_main_screen_update_compass(void);
static void sys_ui_main_screen_update_speed_n_distance(void);
static void sys_ui_main_screen_update_countdown(void);
static void sys_ui_main_screen_cb_settings_btn(lv_event_t *event);
static void sys_ui_main_screen_cb_out_btn(lv_event_t *event);
static void sys_ui_main_screen_cb_time_card(lv_event_t *event);
static void sys_ui_main_screen_cb_distance_card(lv_event_t *event);
static void sys_ui_main_screen_cb_env_card(lv_event_t *event);
// Settings screen
static void sys_ui_settings_screen_create(void);
static void sys_ui_settings_screen_draw(void);
static void sys_ui_settings_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_settings_screen_cb_brightness_slider(lv_event_t *event);
static void sys_ui_settings_screen_cb_color_btn(lv_event_t *event);
// Out screen
static void sys_ui_out_screen_create(void);
static void sys_ui_out_screen_draw(void);
static void sys_ui_out_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_out_screen_cb_confirm_btn(lv_event_t *event);
// Lock screen
static void sys_ui_lock_screen_create(void);
static void sys_ui_lock_screen_draw(void);
#if SCREEN_SKIP_LOCK_SCREEN
static void sys_ui_lock_screen_cb_debug(lv_event_t *event);
#endif
// Time history screen
static void sys_ui_time_screen_create(void);
static void sys_ui_time_screen_update(void);
static void sys_ui_time_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_time_screen_cb_extend_btn(lv_event_t *event);
// Fusion screen
static void sys_ui_fusion_screen_create(void);
static void sys_ui_fusion_screen_update(void);
static void sys_ui_fusion_screen_cb_back_btn(lv_event_t *event);
// Temperature screen
static void sys_ui_temp_screen_create(void);
static void sys_ui_temp_screen_update(void);
static void sys_ui_temp_screen_refresh(void);
static void sys_ui_temp_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_temp_screen_cb_zoom_minus(lv_event_t *event);
static void sys_ui_temp_screen_cb_zoom_plus(lv_event_t *event);
static void sys_ui_temp_screen_cb_pan_left(lv_event_t *event);
static void sys_ui_temp_screen_cb_pan_right(lv_event_t *event);
// History logging
static void sys_ui_log_distance_sample(float value);
static void sys_ui_log_temp_sample(float value, size_t timestamp);
// LVGL driver
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);

/* Function definitions ----------------------------------------------- */
void sys_ui_init(void)
{
  memset(&ui_ctx, 0, sizeof(ui_ctx));

  OS_SEM_CREATE(sys_ui_wakeup_sem);
  ui_ctx.prev_speed_int      = -1;
  ui_ctx.battery_percent     = 85;
  ui_ctx.brightness_percent  = 80;
  ui_ctx.background_color    = SYS_UI_COLOR_BG;
  ui_ctx.session_start_ms    = OS_GET_TICK();
  ui_ctx.view                = SYS_UI_VIEW_LOCK;
  ui_ctx.last_view           = SYS_UI_VIEW_UNKNOWN;
  ui_ctx.pending_main_redraw = false;
  ui_ctx.temperature_zoom    = 1;

  sys_ui_reset_all_widgets(&ui_ctx.widgets);
  bsp_display_init();

  ui_ctx.lvgl.user_data = &ui_ctx;
  lvgl_driver_init(&ui_ctx.lvgl, bsp_display_get_driver());
  lvgl_driver_set_touch_callback(&ui_ctx.lvgl, sys_ui_lvgl_touch_read_cb);

  sys_ui_init_data();
  sys_ui_init_history_data();
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);
  sys_ui_init_all_widgets();
  sys_ui_register_callbacks();

  size_t now                     = OS_GET_TICK();
  ui_ctx.session_start_ms        = now;
  ui_ctx.last_speed_update       = now;
  ui_ctx.last_second_tick        = now;
  ui_ctx.last_time_update_screen = now;

  bsp_touch_init();
  bsp_led_init(SYS_UI_LED_RGB_TASK_MS);

  LOG_DBG("sys_ui_init: complete");
}

void sys_ui_process(void)
{
  switch (g_device_info.state)
  {
  case DEVICE_STATE_IDLE:
  {
    sys_ui_process_idle();
    break;
  }
  case DEVICE_STATE_LOCKED:
  {
    if (ui_ctx.last_view != SYS_UI_VIEW_LOCK)
    {
      sys_ui_change_screen(SYS_UI_VIEW_LOCK);
      ui_ctx.last_view = SYS_UI_VIEW_LOCK;
    }
    sys_ui_process_locked();
    lvgl_driver_task(&ui_ctx.lvgl);
    break;
  }
  case DEVICE_STATE_ACTIVE:
  {
    if (ui_ctx.last_view != SYS_UI_VIEW_MAIN)
    {
      sys_ui_change_screen(SYS_UI_VIEW_MAIN);
      ui_ctx.last_view = SYS_UI_VIEW_MAIN;
    }
    sys_ui_process_active();
    lvgl_driver_task(&ui_ctx.lvgl);
    break;
  }
  default: break;
  }
}

void sys_ui_lock(void)
{
  LOG_DBG("sys_ui_lock: locking");
  ui_ctx.view = SYS_UI_VIEW_UNKNOWN;
}

void sys_ui_unlock(void)
{
  LOG_DBG("sys_ui_unlock: unlocking");
  ui_ctx.view = SYS_UI_VIEW_UNKNOWN;
}

void sys_ui_wakeup(void)
{
  OS_SEM_GIVE(sys_ui_wakeup_sem);
}

/* Private definitions ----------------------------------------------- */
static void sys_ui_show_screen(lv_obj_t *screen)
{
  if (screen == nullptr)
  {
    return;
  }

  ui_ctx.widgets.active_screen = screen;
  lv_screen_load(screen);
}

static void sys_ui_change_screen(sys_ui_view_t view)
{
  ui_ctx.view = view;

  switch (view)
  {
  case SYS_UI_VIEW_LOCK:
  {
    sys_ui_lock_screen_draw();
    break;
  }
  case SYS_UI_VIEW_MAIN:
  {
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
      sys_ui_reset_all_widgets(&ui_ctx.widgets);
    }
    ui_ctx.pending_main_redraw = false;
    sys_ui_init_all_widgets();
    sys_ui_register_callbacks();
    break;
  }
  case SYS_UI_VIEW_SETTINGS:
  {
    sys_ui_settings_screen_draw();
    if (ui_ctx.widgets.settings_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.settings_back_btn, sys_ui_settings_screen_cb_back_btn, LV_EVENT_CLICKED,
                          nullptr);
    }
    if (ui_ctx.widgets.brightness_slider != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.brightness_slider, sys_ui_settings_screen_cb_brightness_slider,
                          LV_EVENT_VALUE_CHANGED, nullptr);
    }
    for (int i = 0; i < SYS_UI_BG_COLOR_COUNT; ++i)
    {
      if (ui_ctx.widgets.color_btns[i] != nullptr)
      {
        lv_obj_add_event_cb(ui_ctx.widgets.color_btns[i], sys_ui_settings_screen_cb_color_btn, LV_EVENT_CLICKED,
                            (void *) (intptr_t) i);
      }
    }
    break;
  }
  case SYS_UI_VIEW_OUT:
  {
    sys_ui_out_screen_draw();
    if (ui_ctx.widgets.out_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.out_back_btn, sys_ui_out_screen_cb_back_btn, LV_EVENT_CLICKED, nullptr);
      lv_obj_add_event_cb(ui_ctx.widgets.out_confirm_btn, sys_ui_out_screen_cb_confirm_btn, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    sys_ui_time_screen_update();
    if (ui_ctx.widgets.time_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.time_back_btn, sys_ui_time_screen_cb_back_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.extend_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.extend_btn, sys_ui_time_screen_cb_extend_btn, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    sys_ui_fusion_screen_update();
    if (ui_ctx.widgets.distance_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.distance_back_btn, sys_ui_fusion_screen_cb_back_btn, LV_EVENT_CLICKED,
                          nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_temp_screen_update();
    if (ui_ctx.widgets.temp_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.temp_back_btn, sys_ui_temp_screen_cb_back_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.zoom_minus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.zoom_minus_btn, sys_ui_temp_screen_cb_zoom_minus, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.zoom_plus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.zoom_plus_btn, sys_ui_temp_screen_cb_zoom_plus, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.pan_left_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.pan_left_btn, sys_ui_temp_screen_cb_pan_left, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.pan_right_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.pan_right_btn, sys_ui_temp_screen_cb_pan_right, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  default: break;
  }
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

  const char *seed[SYS_UI_MAX_RENTAL_HISTORY] = {
    "2026-03-03 09:05 - B-1024",
    "2026-03-02 17:40 - B-2201",
    "2026-03-01 08:15 - B-3108",
    "2026-02-29 19:50 - B-1876",
  };
  ui_ctx.rental_history_count = SYS_UI_MAX_RENTAL_HISTORY;
  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    strncpy(ui_ctx.rental_history[i], seed[i], sizeof(ui_ctx.rental_history[i]) - 1);
    ui_ctx.rental_history[i][sizeof(ui_ctx.rental_history[i]) - 1] = '\0';
  }
}

static void sys_ui_init_history_data(void)
{
  size_t base = OS_GET_TICK();

  for (int i = 0; i < SYS_UI_DATA_HISTORY_SAMPLES; ++i)
  {
    distance_history_data[i]    = 0.2f * (float) i;
    temperature_history_data[i] = 26.0f + 2.0f * sinf((float) i / 10.0f);
    temperature_time_data[i]    = base + (size_t) i * 1000U;
  }
}

static void sys_ui_init_all_widgets(void)
{
  if (ui_ctx.widgets.lock_screen == nullptr)
  {
    sys_ui_lock_screen_create();
  }

  if (ui_ctx.view == SYS_UI_VIEW_LOCK)
  {
    sys_ui_show_screen(ui_ctx.widgets.lock_screen);
    return;
  }

  if (ui_ctx.widgets.main_screen == nullptr)
  {
    sys_ui_main_screen_create();
  }

  sys_ui_show_screen(ui_ctx.widgets.main_screen);
  sys_ui_main_screen_update_compass();
  sys_ui_main_screen_update_time(ui_ctx.remaining_minutes, ui_ctx.remaining_seconds);
  sys_ui_main_screen_update_distance(ui_ctx.distance_km);
  sys_ui_main_screen_update_env(ui_ctx.temperature_C, ui_ctx.humidity, ui_ctx.air_quality);
  sys_ui_main_screen_update_speed(static_cast<int>(ui_ctx.current_speed));
}

static void sys_ui_reset_all_widgets(sys_ui_widgets_t *screen)
{
  memset(screen, 0, sizeof(sys_ui_widgets_t));

  for (int i = 0; i < SYS_UI_BG_COLOR_COUNT; ++i)
  {
    screen->color_btns[i] = nullptr;
  }
  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    screen->history_labels[i] = nullptr;
  }
}

static void sys_ui_register_callbacks(void)
{
  if (ui_ctx.widgets.settings_btn != nullptr)
  {
    lv_obj_add_event_cb(ui_ctx.widgets.settings_btn, sys_ui_main_screen_cb_settings_btn, LV_EVENT_CLICKED, nullptr);
  }
  if (ui_ctx.widgets.out_btn != nullptr)
  {
    lv_obj_add_event_cb(ui_ctx.widgets.out_btn, sys_ui_main_screen_cb_out_btn, LV_EVENT_CLICKED, nullptr);
  }
  if (ui_ctx.widgets.time_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.time_card, sys_ui_main_screen_cb_time_card, nullptr);
  }
  if (ui_ctx.widgets.distance_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.distance_card, sys_ui_main_screen_cb_distance_card, nullptr);
  }
  if (ui_ctx.widgets.env_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.env_card, sys_ui_main_screen_cb_env_card, nullptr);
  }
}

static status_function_t sys_ui_image_draw(lv_obj_t       *parent,
                                           const char     *path,
                                           uint16_t        width,
                                           uint16_t        height,
                                           int16_t         x,
                                           int16_t         y,
                                           lv_image_dsc_t *out_dsc,
                                           lv_obj_t      **out_obj)
{
  if ((parent == nullptr) || (path == nullptr) || (out_dsc == nullptr) || (out_obj == nullptr))
  {
    LOG_ERR("sys_ui_image_draw: invalid arguments");
    return STATUS_ERROR;
  }

  // 1. Allocate PSRAM buffer
  size_t   buf_size = (size_t) width * height * 2U;
  uint8_t *buf      = (uint8_t *) heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  if (buf == nullptr)
  {
    LOG_ERR("sys_ui_image_draw: PSRAM alloc failed (%u bytes)", buf_size);
    return STATUS_ERROR;
  }

  // 2. Read file from SD into PSRAM buffer
  bsp_sdcard_file_t file;
  size_t            read_len = 0U;

  if (bsp_sdcard_open(path, BSP_SDCARD_MODE_READ, &file) != STATUS_OK)
  {
    LOG_ERR("sys_ui_image_draw: cannot open %s", path);
    heap_caps_free(buf);
    return STATUS_ERROR;
  }

  if (bsp_sdcard_read(&file, buf, buf_size, &read_len) != STATUS_OK || read_len != buf_size)
  {
    LOG_ERR("sys_ui_image_draw: read failed, got %u / %u bytes", read_len, buf_size);
    bsp_sdcard_close(&file);
    heap_caps_free(buf);
    return STATUS_ERROR;
  }

  bsp_sdcard_close(&file);

  // 3. Bind buffer
  out_dsc->header.magic  = LV_IMAGE_HEADER_MAGIC;
  out_dsc->header.cf     = LV_COLOR_FORMAT_RGB565;
  out_dsc->header.w      = width;
  out_dsc->header.h      = height;
  out_dsc->header.stride = width * 2U;  // RGB565: 2 bytes/pixel
  out_dsc->data_size     = (uint32_t) buf_size;
  out_dsc->data          = buf;

  // 4. Create LVGL image object and attach descriptor
  *out_obj = lv_image_create(parent);
  if (*out_obj == nullptr)
  {
    LOG_ERR("sys_ui_image_draw: lv_image_create failed");
    heap_caps_free(buf);
    return STATUS_ERROR;
  }

  lv_image_set_src(*out_obj, out_dsc);
  lv_obj_set_pos(*out_obj, x, y);

  LOG_DBG("sys_ui_image_draw: OK %s (%ux%u) pos(%d,%d)", path, width, height, x, y);
  return STATUS_OK;
}

static void sys_ui_main_screen_create(void)
{
  ui_ctx.widgets.main_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  // Top buttons
  ui_ctx.widgets.settings_btn = sys_ui_widget_create_button(ui_ctx.widgets.main_screen, SYS_UI_SETTINGS_BTN_X,
                                                            SYS_UI_CTRL_BTN_Y, SYS_UI_SETTINGS_BTN_W, SYS_UI_CTRL_BTN_H,
                                                            "SETTINGS", SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);
  ui_ctx.widgets.out_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.main_screen, SYS_UI_OUT_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_OUT_BTN_W,
                                SYS_UI_CTRL_BTN_H, "OUT", SYS_UI_COLOR_WARNING, SYS_UI_COLOR_BG);

  // Speedometer arc
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

  // Compass panel
  sys_ui_widget_create_panel(ui_ctx.widgets.main_screen, SYS_UI_MAP_PANEL_X, SYS_UI_MAP_PANEL_Y, SYS_UI_MAP_PANEL_W,
                             SYS_UI_MAP_PANEL_H, SYS_UI_COMPASS_PANEL_BG_COLOR);
  static lv_point_precise_t needle_points[2] = {
    { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY },
    { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY - SYS_UI_COMPASS_R + 4 },
  };
  ui_ctx.widgets.compass_needle = lv_line_create(ui_ctx.widgets.main_screen);
  lv_line_set_points(ui_ctx.widgets.compass_needle, needle_points, 2);
  lv_obj_set_style_line_width(ui_ctx.widgets.compass_needle, 2, 0);
  lv_obj_set_style_line_color(ui_ctx.widgets.compass_needle, lv_color_hex(SYS_UI_COLOR_ACCENT), 0);
  ui_ctx.widgets.compass_deg_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_HEADING_TEXT_X, SYS_UI_HEADING_TEXT_Y, "0°",
                               SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);
  ui_ctx.widgets.compass_dir_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_COMPASS_DEG_X, SYS_UI_COMPASS_DEG_Y, "N",
                               SYS_UI_COLOR_ACCENT, &lv_font_montserrat_18);

  // Time card
  ui_ctx.widgets.time_card = sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_TIME_CARD_Y,
                                                       SYS_UI_CARD_W, SYS_UI_TIME_CARD_H, SYS_UI_COLOR_PRIMARY);
  sys_ui_widget_create_label(ui_ctx.widgets.time_card, 8, 2, "REMAIN", SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);
  ui_ctx.widgets.time_label =
    sys_ui_widget_create_label(ui_ctx.widgets.time_card, 5, 16, "00:00", SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_18);
  ui_ctx.widgets.time_unit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.time_card, 24, 38, "mins", SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Distance card
  ui_ctx.widgets.distance_card =
    sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_DIST_CARD_Y, SYS_UI_CARD_W,
                              SYS_UI_DIST_CARD_H, SYS_UI_COLOR_ACCENT);
  sys_ui_widget_create_label(ui_ctx.widgets.distance_card, 5, 2, "DISTANCE", SYS_UI_COLOR_TEXT_DIM,
                             &lv_font_montserrat_10);
  ui_ctx.widgets.distance_label      = sys_ui_widget_create_label(ui_ctx.widgets.distance_card, 5, 16, "0.00",
                                                                  SYS_UI_COLOR_ACCENT, &lv_font_montserrat_16);
  ui_ctx.widgets.distance_unit_label = sys_ui_widget_create_label(ui_ctx.widgets.distance_card, 62, 20, "km",
                                                                  SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Environment card
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

static void sys_ui_main_screen_update_speed(int speed_kph)
{
  speed_kph = SYS_UI_CLAMP(speed_kph, 0, 40);

  if (ui_ctx.prev_speed_int == speed_kph)
  {
    return;
  }
  ui_ctx.prev_speed_int = speed_kph;

  if (ui_ctx.widgets.speedometer_arc != nullptr)
  {
    int    arc_value = (speed_kph * 100) / 40;
    size_t arc_color = SYS_UI_COLOR_DANGER;
    if (speed_kph <= 20)
      arc_color = SYS_UI_COLOR_SUCCESS;
    else if (speed_kph <= 30)
      arc_color = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_arc_value(ui_ctx.widgets.speedometer_arc, arc_value);
    sys_ui_widget_set_arc_color(ui_ctx.widgets.speedometer_arc, arc_color);
  }

  if (ui_ctx.widgets.speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.speed_label, "%d", speed_kph);
  }
}

static void sys_ui_main_screen_update_time(int minutes, int seconds)
{
  if (ui_ctx.widgets.time_label == nullptr)
  {
    return;
  }

  size_t color = SYS_UI_COLOR_SUCCESS;
  if (minutes < 5)
    color = SYS_UI_COLOR_DANGER;
  else if (minutes < 10)
    color = SYS_UI_COLOR_WARNING;

  sys_ui_widget_set_label_text_format(ui_ctx.widgets.time_label, "%02d:%02d", minutes, seconds);
  sys_ui_widget_set_label_color(ui_ctx.widgets.time_label, color);
}

static void sys_ui_main_screen_update_distance(float distance_km)
{
  if (ui_ctx.widgets.distance_label == nullptr)
  {
    return;
  }

  sys_ui_widget_set_label_text_format(ui_ctx.widgets.distance_label, "%.2f", distance_km);
}

static void sys_ui_main_screen_update_env(float temperature_C, float humidity, int air_quality)
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
    const char *status    = (air_quality >= 80) ? "Good" : ((air_quality >= 50) ? "Fair" : "Poor");
    size_t      aqi_color = SYS_UI_COLOR_DANGER;
    if (air_quality >= 80)
      aqi_color = SYS_UI_COLOR_SUCCESS;
    else if (air_quality >= 50)
      aqi_color = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.aqi_label, "AQI:%d %s", air_quality, status);
    sys_ui_widget_set_label_color(ui_ctx.widgets.aqi_label, aqi_color);
  }
}

static void sys_ui_main_screen_update_compass(void)
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
    lv_point_precise_t points[2] = {
      { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY },
      { x2, y2 },
    };
    lv_line_set_points(ui_ctx.widgets.compass_needle, points, 2);
  }
}

static void sys_ui_main_screen_update_speed_n_distance(void)
{
  ui_ctx.current_speed       = ui_ctx.fusion.velocity_kmh;
  ui_ctx.max_speed           = (ui_ctx.max_speed > ui_ctx.current_speed) ? ui_ctx.max_speed : ui_ctx.current_speed;
  ui_ctx.distance_km         = ui_ctx.fusion.distance_m / 1000.0f;
  ui_ctx.compass_heading_deg = ui_ctx.fusion.heading_deg;
  sys_ui_main_screen_update_speed((int) ui_ctx.current_speed);
  sys_ui_main_screen_update_distance(ui_ctx.distance_km);
}

static void sys_ui_main_screen_update_countdown(void)
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

static void sys_ui_main_screen_cb_settings_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_settings_btn");
  sys_ui_change_screen(SYS_UI_VIEW_SETTINGS);
}

static void sys_ui_main_screen_cb_out_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_out_btn");
  sys_ui_change_screen(SYS_UI_VIEW_OUT);
}

static void sys_ui_main_screen_cb_time_card(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_time_card");
  sys_ui_change_screen(SYS_UI_VIEW_TIME);
}

static void sys_ui_main_screen_cb_distance_card(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_distance_card");
  sys_ui_change_screen(SYS_UI_VIEW_DISTANCE);
}

static void sys_ui_main_screen_cb_env_card(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_env_card");
  sys_ui_change_screen(SYS_UI_VIEW_TEMPERATURE);
}

static void sys_ui_settings_screen_create(void)
{
  ui_ctx.widgets.settings_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.settings_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.settings_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "< BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);
  ui_ctx.widgets.settings_title = sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 90, 13, "SETTINGS",
                                                             SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 40, 52, "Brightness", SYS_UI_COLOR_TEXT_DIM, nullptr);
  ui_ctx.widgets.brightness_slider = sys_ui_widget_create_slider(
    ui_ctx.widgets.settings_screen, 40, 72, 220, 22, 5, 100, ui_ctx.brightness_percent, 0x1A2A3A, SYS_UI_COLOR_PRIMARY);
  ui_ctx.widgets.brightness_label =
    sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 268, 72, "", SYS_UI_COLOR_TEXT, nullptr);
  sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, "%d%%", ui_ctx.brightness_percent);

  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, 40, 108, "Background", SYS_UI_COLOR_TEXT_DIM, nullptr);

  // clang-format off
#define INFO(idx, r, g, b, lbl)                                                                                    \
  ui_ctx.widgets.color_btns[idx] = lv_btn_create(ui_ctx.widgets.settings_screen);                                 \
  lv_obj_set_pos(ui_ctx.widgets.color_btns[idx], 40 + (idx) * SYS_UI_SWATCH_SPAN, SYS_UI_SWATCH_ROW_Y);           \
  lv_obj_set_size(ui_ctx.widgets.color_btns[idx], SYS_UI_SWATCH_SIZE, SYS_UI_SWATCH_SIZE);                        \
  lv_obj_set_style_bg_color(ui_ctx.widgets.color_btns[idx], lv_color_hex(BSP_DISPLAY_RGB_TO_HEX(r, g, b)), 0);    \
  lv_obj_set_style_border_color(ui_ctx.widgets.color_btns[idx], lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0);          \
  lv_obj_set_style_border_width(ui_ctx.widgets.color_btns[idx], 1, 0);                                            \
  lv_obj_set_style_radius(ui_ctx.widgets.color_btns[idx], 4, 0);                                                  \
  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen,                                                      \
    40 + (idx) * SYS_UI_SWATCH_SPAN, SYS_UI_SWATCH_ROW_Y + SYS_UI_SWATCH_SIZE + 3,                               \
    lbl, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  SYS_UI_BG_COLOR_TABLE(INFO)
#undef INFO
  // clang-format on
}

static void sys_ui_settings_screen_draw(void)
{
  if (ui_ctx.widgets.settings_screen == nullptr)
  {
    sys_ui_settings_screen_create();
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

static void sys_ui_settings_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_settings_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_settings_screen_cb_brightness_slider(lv_event_t *event)
{
  lv_obj_t *slider          = (lv_obj_t *) lv_event_get_target(event);
  ui_ctx.brightness_percent = lv_slider_get_value(slider);
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);

  if (ui_ctx.widgets.brightness_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, "%d%%", ui_ctx.brightness_percent);
  }

  LOG_DBG("sys_ui_settings_screen_cb_brightness_slider: %d%%", ui_ctx.brightness_percent);
}

static void sys_ui_settings_screen_cb_color_btn(lv_event_t *event)
{
  int btn_index = (int) (intptr_t) lv_event_get_user_data(event);

#define INFO(idx, r, g, b, lbl) BSP_DISPLAY_RGB_TO_HEX(r, g, b),
  static const size_t bg_colors[SYS_UI_BG_COLOR_COUNT] = { SYS_UI_BG_COLOR_TABLE(INFO) };
#undef INFO

  if (btn_index >= 0 && btn_index < SYS_UI_BG_COLOR_COUNT)
  {
    ui_ctx.background_color    = bg_colors[btn_index];
    ui_ctx.pending_main_redraw = true;
    LOG_DBG("sys_ui_settings_screen_cb_color_btn: index=%d", btn_index);
  }
}

static void sys_ui_out_screen_create(void)
{
  ui_ctx.widgets.out_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.out_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.out_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, SYS_UI_BACK_BTN_LABEL, SYS_UI_BACK_BTN_COLOR, SYS_UI_COLOR_BG);
  sys_ui_widget_create_label(ui_ctx.widgets.out_screen, SYS_UI_OUT_LABEL_X, SYS_UI_OUT_LABEL_Y, SYS_UI_OUT_LABEL_TEXT,
                             SYS_UI_COLOR_TEXT, nullptr);
  ui_ctx.widgets.out_confirm_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.out_screen, SYS_UI_CONFIRM_BTN_X, SYS_UI_CONFIRM_BTN_Y, SYS_UI_CONFIRM_BTN_W, SYS_UI_CONFIRM_BTN_H,
    SYS_UI_CONFIRM_BTN_LABEL, SYS_UI_CONFIRM_BTN_COLOR, SYS_UI_COLOR_BG);
}

static void sys_ui_out_screen_draw(void)
{
  if (ui_ctx.widgets.out_screen == nullptr)
  {
    sys_ui_out_screen_create();
  }

  sys_ui_show_screen(ui_ctx.widgets.out_screen);
}

static void sys_ui_out_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_out_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_out_screen_cb_confirm_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_out_screen_cb_confirm_btn");
  sys_manager_write_event(SYS_MANAGER_EVT_USER_LOCK);
}

static void sys_ui_time_screen_create(void)
{
  ui_ctx.widgets.time_history_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.time_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.time_history_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y,
                                SYS_UI_BACK_BTN_W, SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);
  ui_ctx.widgets.time_history_title = sys_ui_widget_create_label(
    ui_ctx.widgets.time_history_screen, 60, 15, "RENTAL HISTORY", SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
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

static void sys_ui_time_screen_update(void)
{
  if (ui_ctx.widgets.time_history_screen == nullptr)
  {
    sys_ui_time_screen_create();
  }

  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    if (ui_ctx.widgets.history_labels[i] == nullptr)
    {
      continue;
    }
    const char *text = (i < ui_ctx.rental_history_count) ? ui_ctx.rental_history[i] : "";
    sys_ui_widget_set_label_text(ui_ctx.widgets.history_labels[i], text);
  }

  if (ui_ctx.widgets.time_remaining_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.time_remaining_label, "Remaining: %02d:%02d",
                                        ui_ctx.remaining_minutes, ui_ctx.remaining_seconds);
  }

  sys_ui_show_screen(ui_ctx.widgets.time_history_screen);
}

static void sys_ui_time_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_time_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_time_screen_cb_extend_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_time_screen_cb_extend_btn");
  ui_ctx.remaining_minutes += 30;
  sys_ui_time_screen_update();
}

static void sys_ui_fusion_screen_create(void)
{
  ui_ctx.widgets.fusion_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.distance_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.fusion_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, "BACK", SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);
  ui_ctx.widgets.distance_title =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_LABEL_X, SYS_UI_FUSION_LABEL_Y,
                               SYS_UI_FUSION_LABEL_TEXT, SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);
  ui_ctx.widgets.max_speed_lable =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_MAX_SPEED_X, SYS_UI_FUSION_MAX_SPEED_Y,
                               "Total: 0.00 km", SYS_UI_COLOR_TEXT, nullptr);
  ui_ctx.widgets.avg_speed_label =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_AVG_SPEED_X, SYS_UI_FUSION_AVG_SPEED_Y,
                               "Avg speed: 0.0 km/h", SYS_UI_COLOR_TEXT, nullptr);
  ui_ctx.widgets.distance_chart = sys_ui_widget_create_chart(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X, SYS_UI_FUSION_CHART_Y, SYS_UI_FUSION_CHART_W,
    SYS_UI_FUSION_CHART_H, SYS_UI_FUSION_CHART_POINTS, SYS_UI_FUSION_CHART_COLOR);
  ui_ctx.widgets.distance_series = sys_ui_widget_add_chart_series(ui_ctx.widgets.distance_chart, SYS_UI_COLOR_ACCENT);
}

static void sys_ui_fusion_screen_update(void)
{
  if (ui_ctx.widgets.fusion_screen == nullptr)
  {
    sys_ui_fusion_screen_create();
  }

  size_t now   = OS_GET_TICK();
  float  hours = (now - ui_ctx.session_start_ms) / 3600000.0f;
  if (hours < 0.001f)
  {
    hours = 0.001f;
  }

  if (ui_ctx.widgets.max_speed_lable != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.max_speed_lable, SYS_UI_FUSION_MAX_SPEED_TEXT, ui_ctx.max_speed);
  }
  if (ui_ctx.widgets.avg_speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.avg_speed_label, SYS_UI_FUSION_AVG_SPEED_TEXT,
                                        ui_ctx.distance_km / hours);
  }

  if (ui_ctx.widgets.distance_chart != nullptr && ui_ctx.widgets.distance_series != nullptr)
  {
    int count = distance_history_count;
    int base  = (count > 16) ? (count - 16) : 0;
    int plot  = (count - base > 0) ? (count - base) : 1;
    lv_chart_set_point_count(ui_ctx.widgets.distance_chart, plot);
    for (int i = 0; i < plot; ++i)
    {
      lv_chart_set_value_by_id(ui_ctx.widgets.distance_chart, ui_ctx.widgets.distance_series, i,
                               (int) (distance_history_data[base + i] * 100));
    }
    lv_chart_refresh(ui_ctx.widgets.distance_chart);
  }

  sys_ui_show_screen(ui_ctx.widgets.fusion_screen);
}

static void sys_ui_fusion_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_fusion_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_temp_screen_create(void)
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
  ui_ctx.widgets.temp_series = sys_ui_widget_add_chart_series(ui_ctx.widgets.temp_chart, SYS_UI_COLOR_ACCENT);
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

static void sys_ui_temp_screen_update(void)
{
  // 1. Create screen if not exists
  if (ui_ctx.widgets.temp_screen == nullptr)
  {
    sys_ui_temp_screen_create();
  }

  // 2. If no data, just show screen
  int count = temperature_history_count;
  if (ui_ctx.widgets.temp_chart == nullptr || ui_ctx.widgets.temp_series == nullptr || count <= 1)
  {
    sys_ui_show_screen(ui_ctx.widgets.temp_screen);
    return;
  }

  int zoom         = ui_ctx.temperature_zoom;
  int graph_offset = SYS_UI_CLAMP(ui_ctx.temperature_graph_offset, 0, count - 1);
  int window       = SYS_UI_CLAMP(count / zoom, 2, count);
  graph_offset     = SYS_UI_CLAMP(graph_offset, 0, count - window);

  float min_temp = temperature_history_data[graph_offset];
  float max_temp = temperature_history_data[graph_offset];
  for (int i = 0; i < window; ++i)
  {
    float now = temperature_history_data[graph_offset + i];
    if (now < min_temp)
      min_temp = now;
    if (now > max_temp)
      max_temp = now;
  }

  if (ui_ctx.widgets.temp_range_label != nullptr)
  {
    float offset = (temperature_time_data[graph_offset + window - 1] - temperature_time_data[graph_offset]) / 1000.0f;
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.temp_range_label, "%.1f-%.1fC (%.1fs)", min_temp, max_temp,
                                        offset);
  }

  lv_chart_set_point_count(ui_ctx.widgets.temp_chart, window);
  lv_chart_set_range(ui_ctx.widgets.temp_chart, LV_CHART_AXIS_PRIMARY_Y, (int) (min_temp * 10) - 5,
                     (int) (max_temp * 10) + 5);
  for (int i = 0; i < window; ++i)
  {
    lv_chart_set_value_by_id(ui_ctx.widgets.temp_chart, ui_ctx.widgets.temp_series, i,
                             (int) (temperature_history_data[graph_offset + i] * 10));
  }
  lv_chart_refresh(ui_ctx.widgets.temp_chart);

  sys_ui_show_screen(ui_ctx.widgets.temp_screen);
}

static void sys_ui_temp_screen_refresh(void)
{
  int max_pan                     = (temperature_history_count > 2) ? (temperature_history_count - 2) : 0;
  ui_ctx.temperature_graph_offset = SYS_UI_CLAMP(ui_ctx.temperature_graph_offset, 0, max_pan);
  sys_ui_temp_screen_update();
}

static void sys_ui_temp_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_temp_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_temp_screen_cb_zoom_minus(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.temperature_zoom > 1)
  {
    --ui_ctx.temperature_zoom;
    sys_ui_temp_screen_refresh();
    LOG_DBG("sys_ui_temp_screen_cb_zoom_minus: zoom=%d", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_temp_screen_cb_zoom_plus(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.temperature_zoom < 4)
  {
    ++ui_ctx.temperature_zoom;
    sys_ui_temp_screen_refresh();
    LOG_DBG("sys_ui_temp_screen_cb_zoom_plus: zoom=%d", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_temp_screen_cb_pan_left(lv_event_t *event)
{
  (void) event;
  ui_ctx.temperature_graph_offset -= 5;
  sys_ui_temp_screen_refresh();
  LOG_DBG("sys_ui_temp_screen_cb_pan_left: pan=%d", ui_ctx.temperature_graph_offset);
}

static void sys_ui_temp_screen_cb_pan_right(lv_event_t *event)
{
  (void) event;
  ui_ctx.temperature_graph_offset += 5;
  sys_ui_temp_screen_refresh();
  LOG_DBG("sys_ui_temp_screen_cb_pan_right: pan=%d", ui_ctx.temperature_graph_offset);
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

static void sys_ui_lock_screen_create(void)
{
  char buf[128];
  ui_ctx.widgets.lock_screen = sys_ui_widget_create_screen(0x000000);

  snprintf(buf, sizeof(buf), "Device ID: %s", g_device_info.device_name);
  sys_ui_widget_create_label(ui_ctx.widgets.lock_screen, SYS_UI_QR_LABEL_X, SYS_UI_QR_LABEL_Y, SYS_UI_QR_LABEL,
                             SYS_UI_COLOR_TEXT, SYS_UI_QR_LABEL_FONT);
  if (sys_ui_image_draw(ui_ctx.widgets.lock_screen, SYS_UI_QR_PATH, SYS_UI_QR_WIDTH, SYS_UI_QR_HEIGHT, SYS_UI_QR_X,
                        SYS_UI_QR_Y, &ui_ctx.widgets.lock_qr_dsc, &ui_ctx.widgets.lock_qr_img)
      != STATUS_OK)
  {
    LOG_ERR("sys_ui_lock_screen_create: QR image load failed");
    ui_ctx.widgets.lock_qr_img =
      sys_ui_widget_create_label(ui_ctx.widgets.lock_screen, SYS_UI_QR_LABEL_FAIL_X, SYS_UI_QR_LABEL_FAIL_Y,
                                 SYS_UI_QR_LABEL_FAIL, SYS_UI_COLOR_TEXT_DIM, SYS_UI_QR_LABEL_FAIL_FONT);
  }
  ui_ctx.widgets.device_id_label =
    sys_ui_widget_create_label(ui_ctx.widgets.lock_screen, SYS_UI_DEVICE_ID_LABEL_X, SYS_UI_DEVICE_ID_LABEL_Y, buf,
                               SYS_UI_COLOR_TEXT, &lv_font_montserrat_10);

#if SCREEN_SKIP_LOCK_SCREEN
  if (ui_ctx.widgets.lock_qr_img != nullptr)
  {
    lv_obj_add_flag(ui_ctx.widgets.lock_qr_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui_ctx.widgets.lock_qr_img, sys_ui_lock_screen_cb_debug, LV_EVENT_CLICKED, nullptr);
  }
#endif
}

static void sys_ui_lock_screen_draw(void)
{
  if (ui_ctx.widgets.lock_screen == nullptr)
  {
    sys_ui_lock_screen_create();
  }

  sys_ui_show_screen(ui_ctx.widgets.lock_screen);
}

static void sys_ui_process_active(void)
{
  size_t now = OS_GET_TICK();

  switch (ui_ctx.view)
  {
  case SYS_UI_VIEW_MAIN:
  {
    if (now - ui_ctx.last_second_tick >= SYS_UI_COUNTDOWN_MS)
    {
      ui_ctx.last_second_tick = now;
      sys_ui_main_screen_update_countdown();
      sys_ui_main_screen_update_time(ui_ctx.remaining_minutes, ui_ctx.remaining_seconds);
    }

    if (g_sys_ui_data_status.is_fusion_data_ready_for_ui)
    {
      sys_input_get_fusion_data(&ui_ctx.fusion);
      sys_ui_main_screen_update_speed_n_distance();
      sys_ui_main_screen_update_compass();
      ui_ctx.frame_counter++;

      if ((ui_ctx.frame_counter % 60U) == 0U)
      {
        sys_ui_log_distance_sample(ui_ctx.distance_km);
      }

      g_sys_ui_data_status.is_fusion_data_ready_for_ui = false;
    }

    if (g_sys_ui_data_status.is_dust_data_ready_for_ui || g_sys_ui_data_status.is_temp_hum_data_ready_for_ui)
    {
      sys_input_data_t env = { 0 };
      if (sys_input_get_env_data(&env) == STATUS_OK)
      {
        ui_ctx.temperature_C = env.temp_hum.temperature;
        ui_ctx.humidity      = env.temp_hum.humidity;
        ui_ctx.air_quality   = bsp_dust_sensor_get_aqi_level(env.dust_value);
      }
      sys_ui_main_screen_update_env(ui_ctx.temperature_C, ui_ctx.humidity, ui_ctx.air_quality);
      g_sys_ui_data_status.is_dust_data_ready_for_ui     = false;
      g_sys_ui_data_status.is_temp_hum_data_ready_for_ui = false;
    }
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    if (now - ui_ctx.last_second_tick >= SYS_UI_COUNTDOWN_MS)
    {
      ui_ctx.last_second_tick = now;
      sys_ui_main_screen_update_countdown();
      sys_ui_time_screen_update();
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    sys_ui_fusion_screen_update();
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_temp_screen_update();
    break;
  }
  case SYS_UI_VIEW_LOCK:
  case SYS_UI_VIEW_SETTINGS:
  case SYS_UI_VIEW_OUT:
  {
    // Static screens — no periodic update needed
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
    sys_ui_reset_all_widgets(&ui_ctx.widgets);
    sys_ui_init_all_widgets();
    sys_ui_register_callbacks();
    ui_ctx.pending_main_redraw = false;
  }
}

static void sys_ui_process_idle(void)
{
  bsp_display_set_brightness_percent(SYS_UI_BRIGHTNESS_PERCENT_OFF);
  OS_SEM_TAKE(sys_ui_wakeup_sem, OS_MAX_DELAY);
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);
}
static void sys_ui_process_locked(void)
{
  // Do nothing, just show the lock screen
}

#if SCREEN_SKIP_LOCK_SCREEN
static void sys_ui_lock_screen_cb_debug(lv_event_t *event)
{
  (void) event;
  sys_manager_write_event(SYS_MANAGER_EVT_UNLOCKED);
}
#endif

/* End of file -------------------------------------------------------- */