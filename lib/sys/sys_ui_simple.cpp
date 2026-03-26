/**
 * @file       sys_ui_simple.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     Simple UI screen creation functions for quick prototyping.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_ui_simple.h"

#include "bsp_display.h"
#include "bsp_led.h"
#include "bsp_touch.h"
#include "log_service.h"
#include "lvgl_driver.h"
#include "os_lib.h"
#include "sys_input.h"

/* Private defines ---------------------------------------------------- */
#define SCREEN_BRIGHTLESS (80)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  lvgl_driver_t lvgl;

  /* Main screen */
  lv_obj_t *ui_screen1         = NULL;
  lv_obj_t *ui_velocity_label  = NULL;
  lv_obj_t *ui_position_label  = NULL;
  lv_obj_t *ui_direction_label = NULL;
  lv_obj_t *ui_temp_label      = NULL;
  lv_obj_t *ui_hum_label       = NULL;
  lv_obj_t *ui_dust_label      = NULL;
  lv_obj_t *ui_distance_label  = NULL;

  lv_obj_t *ui_velocity_value  = NULL;
  lv_obj_t *ui_position_lat    = NULL;
  lv_obj_t *ui_position_long   = NULL;
  lv_obj_t *ui_direction_value = NULL;
  lv_obj_t *ui_direction_str   = NULL;
  lv_obj_t *ui_temp_value      = NULL;
  lv_obj_t *ui_hum_value       = NULL;
  lv_obj_t *ui_dust_value      = NULL;
  lv_obj_t *ui_distance_value  = NULL;

  lv_obj_t *ui_Login    = NULL;
  lv_obj_t *ui_LoginBtn = NULL;

  lv_obj_t *ui_BackBtn = NULL;
} sys_ui_simple_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
bool is_ui_simple_data_ready = false;

/* Private variables -------------------------------------------------- */
static sys_ui_simple_t  ui_ctx;
static sys_input_data_t input_data;

/* Private function prototypes ---------------------------------------- */
static void ui_main_screen_init(void);
static void ui_Login_screen_init(void);
static void login_btn_event_cb(lv_event_t *e);
static void back_btn_event_cb(lv_event_t *e);
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);
static void sys_ui_simple_update_values(void);

/* Function definitions ----------------------------------------------- */
void sys_ui_simple_init(void)
{
  // Led initialization
  bsp_led_init(SYS_UI_TASK_SLEEP_MS);

  // Int first display
  bsp_display_init();

  // Initialize LVGL driver
  ui_ctx.lvgl.user_data = &ui_ctx;
  lvgl_driver_init(&ui_ctx.lvgl, bsp_display_get_driver());
  lvgl_driver_set_touch_callback(&ui_ctx.lvgl, sys_ui_lvgl_touch_read_cb);

  /* Set brightness */
  bsp_display_set_brightness_percent(SCREEN_BRIGHTLESS);

  // Create login screen first
  ui_Login_screen_init();
  ui_main_screen_init();
  lv_disp_load_scr(ui_ctx.ui_Login);

  /* Initialize touch */
  bsp_touch_init();

  lv_display_set_rotation(ui_ctx.lvgl.display, LV_DISPLAY_ROTATION_0);
}
void sys_ui_simple_process(void)
{
  lvgl_driver_task(&ui_ctx.lvgl);
  if (is_ui_simple_data_ready)
  {
    is_ui_simple_data_ready = false;
    sys_input_get_data(&input_data);
    sys_ui_simple_update_values();
  }
  bsp_led_task();
}

void sys_ui_simple_change_ui(sys_ui_simple_view_t view)
{
  switch (view)
  {
  case SYS_UI_SIMPLE_VIEW_MAIN:
  {
    lv_disp_load_scr(ui_ctx.ui_screen1);
    break;
  }
  case SYS_UI_SIMPLE_VIEW_LOGIN:
  {
    lv_disp_load_scr(ui_ctx.ui_Login);
    break;
  }
  default: break;
  }
}

/* Private definitions ----------------------------------------------- */

static void ui_main_screen_init(void)
{
  ui_ctx.ui_screen1 = lv_obj_create(NULL);
  lv_obj_clear_flag(ui_ctx.ui_screen1, LV_OBJ_FLAG_SCROLLABLE);

  int start_y = 20;
  int row_h   = 30;
  int label_x = 10;
  int value_x = 120;

  ui_ctx.ui_velocity_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_velocity_label, label_x, start_y + 0 * row_h);
  lv_label_set_text(ui_ctx.ui_velocity_label, "Velocity(m/s)");
  ui_ctx.ui_velocity_value = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_velocity_value, value_x, start_y + 0 * row_h);
  lv_label_set_text(ui_ctx.ui_velocity_value, "0");

  ui_ctx.ui_position_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_position_label, label_x, start_y + 1 * row_h);
  lv_label_set_text(ui_ctx.ui_position_label, "Position");
  ui_ctx.ui_position_long = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_position_long, value_x, start_y + 1 * row_h);
  lv_label_set_text(ui_ctx.ui_position_long, "0.0000, 0.0000");

  ui_ctx.ui_distance_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_distance_label, label_x, start_y + 2 * row_h);
  lv_label_set_text(ui_ctx.ui_distance_label, "Distance(m)");
  ui_ctx.ui_distance_value = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_distance_value, value_x, start_y + 2 * row_h);
  lv_label_set_text(ui_ctx.ui_distance_value, "0");

  ui_ctx.ui_direction_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_direction_label, label_x, start_y + 3 * row_h);
  lv_label_set_text(ui_ctx.ui_direction_label, "Direction");
  ui_ctx.ui_direction_value = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_direction_value, value_x, start_y + 3 * row_h);
  lv_label_set_text(ui_ctx.ui_direction_value, "0");
  ui_ctx.ui_direction_str = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_direction_str, value_x + 50, start_y + 3 * row_h);
  lv_label_set_text(ui_ctx.ui_direction_str, "N");

  ui_ctx.ui_temp_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_temp_label, label_x, start_y + 4 * row_h);
  lv_label_set_text(ui_ctx.ui_temp_label, "Temp(C)");
  ui_ctx.ui_temp_value = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_temp_value, value_x, start_y + 4 * row_h);
  lv_label_set_text(ui_ctx.ui_temp_value, "0");

  ui_ctx.ui_hum_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_hum_label, label_x, start_y + 5 * row_h);
  lv_label_set_text(ui_ctx.ui_hum_label, "Hum(%)");
  ui_ctx.ui_hum_value = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_hum_value, value_x, start_y + 5 * row_h);
  lv_label_set_text(ui_ctx.ui_hum_value, "0");

  ui_ctx.ui_dust_label = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_dust_label, label_x, start_y + 6 * row_h);
  lv_label_set_text(ui_ctx.ui_dust_label, "Dust(ug/m^3)");
  ui_ctx.ui_dust_value = lv_label_create(ui_ctx.ui_screen1);
  lv_obj_set_pos(ui_ctx.ui_dust_value, value_x, start_y + 6 * row_h);
  lv_label_set_text(ui_ctx.ui_dust_value, "0");

  ui_ctx.ui_BackBtn = lv_btn_create(ui_ctx.ui_screen1);
  lv_obj_set_size(ui_ctx.ui_BackBtn, 60, 30);
  lv_obj_align(ui_ctx.ui_BackBtn, LV_ALIGN_BOTTOM_RIGHT, -10, -12);
  lv_obj_t *back_label = lv_label_create(ui_ctx.ui_BackBtn);
  lv_label_set_text(back_label, "Back");
  lv_obj_center(back_label);
  lv_obj_add_event_cb(ui_ctx.ui_BackBtn, back_btn_event_cb, LV_EVENT_CLICKED, NULL);
}

static void ui_Login_screen_init(void)
{
  ui_ctx.ui_Login = lv_obj_create(NULL);
  lv_obj_clear_flag(ui_ctx.ui_Login, LV_OBJ_FLAG_SCROLLABLE);
  // Button
  ui_ctx.ui_LoginBtn = lv_btn_create(ui_ctx.ui_Login);
  lv_obj_set_size(ui_ctx.ui_LoginBtn, 120, 50);
  lv_obj_center(ui_ctx.ui_LoginBtn);
  lv_obj_t *label = lv_label_create(ui_ctx.ui_LoginBtn);
  lv_label_set_text(label, "Login");
  lv_obj_center(label);
  lv_obj_add_event_cb(ui_ctx.ui_LoginBtn, login_btn_event_cb, LV_EVENT_CLICKED, NULL);
}

static void login_btn_event_cb(lv_event_t *e)
{
  lv_disp_load_scr(ui_ctx.ui_screen1);
}

static void back_btn_event_cb(lv_event_t *e)
{
  lv_disp_load_scr(ui_ctx.ui_Login);
}

static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  sys_ui_simple_t *ctx = (sys_ui_simple_t *) lv_indev_get_user_data(indev);
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

static void sys_ui_simple_update_values(void)
{
  char buf[32];

  float       velocity  = input_data.velocity_ms;
  float       lat       = input_data.gps_position.latitude;
  float       lon       = input_data.gps_position.longitude;
  float       direction = input_data.heading_deg;
  const char *dir_str   = input_data.direction_str;
  float       temp      = input_data.temp_hum.temperature;
  float       hum       = input_data.temp_hum.humidity;
  float       dust      = input_data.dust_value;
  float       distance  = input_data.distance_m;

  snprintf(buf, sizeof(buf), "%.2f", velocity);
  lv_label_set_text(ui_ctx.ui_velocity_value, buf);
  snprintf(buf, sizeof(buf), "%.6f, %.6f", lat, lon);
  lv_label_set_text(ui_ctx.ui_position_long, buf);
  snprintf(buf, sizeof(buf), "%.1f", direction);
  lv_label_set_text(ui_ctx.ui_direction_value, buf);
  lv_label_set_text(ui_ctx.ui_direction_str, dir_str);
  snprintf(buf, sizeof(buf), "%.2f", temp);
  lv_label_set_text(ui_ctx.ui_temp_value, buf);
  snprintf(buf, sizeof(buf), "%.2f", hum);
  lv_label_set_text(ui_ctx.ui_hum_value, buf);
  snprintf(buf, sizeof(buf), "%.3f", dust);
  lv_label_set_text(ui_ctx.ui_dust_value, buf);
  snprintf(buf, sizeof(buf), "%.1f", distance);
  lv_label_set_text(ui_ctx.ui_distance_value, buf);
}

/* End of file -------------------------------------------------------- */
