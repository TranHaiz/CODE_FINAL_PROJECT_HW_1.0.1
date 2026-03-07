/**
 * @file       sys_ui.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0 (LVGL Full Integration)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      UI controller using LVGL for rendering and event handling.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_ui.h"

#include "os_lib.h"

#include <string.h>


/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
sys_ui_t g_ui;

/* Private variables -------------------------------------------------- */
/* Global context pointer for LVGL callbacks */
static sys_ui_t *g_ui_ctx = nullptr;

/* Forward declarations for LVGL callbacks */
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
static void sys_ui_register_lvgl_callbacks(sys_ui_t *ctx);

/* Private function prototypes ---------------------------------------- */
static void sys_ui_initializeTelemetry(sys_ui_t *ctx);
static void sys_ui_updateSpeedAndDistance(sys_ui_t *ctx);
static void sys_ui_updateCountdown(sys_ui_t *ctx);
static void sys_ui_updateEnvironmentTelemetry(sys_ui_t *ctx);
static void sys_ui_enterView(sys_ui_t *ctx, sys_ui_view_t view);
static void sys_ui_returnToMain(sys_ui_t *ctx);
static void sys_ui_logDistanceSample(sys_ui_t *ctx, float value);
static void sys_ui_logTemperatureSample(sys_ui_t *ctx, float value, uint32_t timestamp);
static void sys_ui_refreshTemperatureOverlay(sys_ui_t *ctx);

/* Function definitions ----------------------------------------------- */
void sys_ui_begin(sys_ui_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  ctx->last_speed_update        = 0;
  ctx->last_second_tick         = 0;
  ctx->last_env_update          = 0;
  ctx->frame_counter            = 0;
  ctx->current_speed            = 0.0f;
  ctx->target_speed             = 0.0f;
  ctx->distance_km              = 0.0f;
  ctx->remaining_minutes        = 0;
  ctx->remaining_seconds        = 0;
  ctx->temperature_C            = 0.0f;
  ctx->humidity                 = 0.0f;
  ctx->air_quality              = 0;
  ctx->battery_percent          = 85;
  ctx->brightness_percent       = 80;
  ctx->theme_background_color   = bsp_display_cvt_background_color(&ctx->display);
  ctx->session_start_ms         = OS_GET_TICK();
  ctx->view                     = SYS_UI_VIEW_MAIN;
  ctx->last_touch_x             = 0;
  ctx->last_touch_y             = 0;
  ctx->pending_main_redraw      = false;
  ctx->rental_history_count     = 0;
  ctx->distance_history_count   = 0;
  ctx->temperature_sample_count = 0;
  ctx->temperature_zoom         = 1;
  ctx->temperature_pan          = 0;

  printf("[SYS_UI] Starting initialization...\n");

  bsp_display_initContext(&ctx->display);
  bsp_display_initialize(&ctx->display);

  // Show splash screen while initializing
  bsp_display_show_splash(&ctx->display);

  // Initialize LVGL driver and set up touch callback
  ctx->lvgl.user_data = ctx; /* Store context for touch callback */

  /* Initialize LVGL with TFT_eSPI driver (passive mode - no auto-refresh) */
  bsp_lvgl_init(&ctx->lvgl, bsp_display_driver(&ctx->display));

  /* Register LVGL touch callback */
  bsp_lvgl_set_touch_callback(&ctx->lvgl, sys_ui_lvgl_touch_read_cb);

  sys_ui_initializeTelemetry(ctx);

  bsp_display_set_brightness_percent(&ctx->display, ctx->brightness_percent);
  bsp_display_set_background_color(&ctx->display, ctx->theme_background_color);
  bsp_display_set_battery_percent(&ctx->display, ctx->battery_percent);

  bsp_display_draw_full_ui(&ctx->display, ctx->remaining_minutes, ctx->remaining_seconds, ctx->distance_km,
                           ctx->current_speed, ctx->temperature_C, ctx->humidity, ctx->air_quality);

  /* Store global context for LVGL callbacks */
  g_ui_ctx = ctx;

  /* Register LVGL button event callbacks */
  sys_ui_register_lvgl_callbacks(ctx);

  ctx->view              = SYS_UI_VIEW_MAIN;
  ctx->session_start_ms  = OS_GET_TICK();
  uint32_t now           = ctx->session_start_ms;
  ctx->last_speed_update = now;
  ctx->last_second_tick  = now;
  ctx->last_env_update   = now;

  bsp_touch_init(&ctx->touch, bsp_display_driver(&ctx->display));
}

void sys_ui_run(sys_ui_t *ctx)
{
  assert(ctx != nullptr);

  uint32_t now = OS_GET_TICK();

  // Update speed and distance
  if (now - ctx->last_speed_update >= SYS_UI_SPEED_REFRESH_MS)
  {
    ctx->last_speed_update = now;
    ++ctx->frame_counter;
    sys_ui_updateSpeedAndDistance(ctx);
  }

  // Update countdown timer
  if (now - ctx->last_second_tick >= SYS_UI_COUNTDOWN_MS)
  {
    ctx->last_second_tick = now;
    sys_ui_updateCountdown(ctx);
  }

  // Update environment telemetry
  if (now - ctx->last_env_update >= SYS_UI_ENVIRONMENT_MS)
  {
    ctx->last_env_update = now;
    sys_ui_updateEnvironmentTelemetry(ctx);
  }

  /* Handle pending main screen redraw if returning from overlay */
  if (ctx->view == SYS_UI_VIEW_MAIN && ctx->pending_main_redraw)
  {
    bsp_display_draw_full_ui(&ctx->display, ctx->remaining_minutes, ctx->remaining_seconds, ctx->distance_km,
                             ctx->current_speed, ctx->temperature_C, ctx->humidity, ctx->air_quality);
    sys_ui_register_lvgl_callbacks(ctx);
    ctx->pending_main_redraw = false;
  }

  /* Handle LVGL tasks (rendering + input handling) */
  bsp_lvgl_task(&ctx->lvgl);
}

/* Private definitions ----------------------------------------------- */
static void sys_ui_initializeTelemetry(sys_ui_t *ctx)
{
  ctx->remaining_minutes = 45;
  ctx->remaining_seconds = 0;
  ctx->current_speed     = 0.0f;
  ctx->target_speed      = 12.0f;
  ctx->distance_km       = 0.0f;

  // NOTE: Randomized baseline telemetry until actual temp sensor is connected.
  ctx->temperature_C = 28.0f + static_cast<float>(random(0, 50)) / 10.0f;
  // NOTE: Randomized humidity placeholder, replace once humidity sensor is wired.
  ctx->humidity = 60.0f + static_cast<float>(random(0, 300)) / 10.0f;
  // NOTE: Randomized AQI placeholder until air-quality module is available.
  ctx->air_quality = 70 + random(0, 30);
  // NOTE: Randomized battery percentage to mimic pack feedback for now.
  ctx->battery_percent = 80 + random(0, 20);

  // NOTE: Placeholder rental history until backend sync is available.
  const char *seedHistory[SYS_UI_MAX_RENTAL_HISTORY] = { "2026-03-03 09:05 - B-1024", "2026-03-02 17:40 - B-2201",
                                                         "2026-03-01 08:15 - B-3108", "2026-02-29 19:50 - B-1876" };
  ctx->rental_history_count                          = SYS_UI_MAX_RENTAL_HISTORY;
  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    strncpy(ctx->rentalHistory[i], seedHistory[i], sizeof(ctx->rentalHistory[i]) - 1);
    ctx->rentalHistory[i][sizeof(ctx->rentalHistory[i]) - 1] = '\0';
  }
}

static void sys_ui_updateSpeedAndDistance(sys_ui_t *ctx)
{
  // NOTE: Randomized target speed simulates wheel sensor data until the CAN feed is ready.
  float speedDelta = static_cast<float>(random(-25, 26)) / 10.0f;
  ctx->target_speed += speedDelta;
  if (ctx->target_speed < 0.0f)
    ctx->target_speed = 0.0f;
  if (ctx->target_speed > 35.0f)
    ctx->target_speed = 35.0f;

  ctx->current_speed += (ctx->target_speed - ctx->current_speed) * 0.2f;

  ctx->distance_km += ctx->current_speed * (SYS_UI_SPEED_REFRESH_MS / 1000.0f) / 3600.0f;

  if (ctx->view == SYS_UI_VIEW_MAIN)
  {
    bsp_display_updateSpeed(&ctx->display, static_cast<int>(ctx->current_speed));
    if (ctx->frame_counter % 10 == 0)
    {
      bsp_display_updateDistance(&ctx->display, ctx->distance_km);
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
    uint32_t now   = OS_GET_TICK();
    float    hours = (now - ctx->session_start_ms) / 3600000.0f;
    if (hours < 0.001f)
      hours = 0.001f;
    float avgSpeed = ctx->distance_km / hours;
    bsp_display_drawDistanceOverlay(&ctx->display, avgSpeed, ctx->distance_km, ctx->distanceHistory,
                                    ctx->distance_history_count);
  }
}

static void sys_ui_updateCountdown(sys_ui_t *ctx)
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
    bsp_display_updateTime(&ctx->display, ctx->remaining_minutes, ctx->remaining_seconds);
  }
  else if (ctx->view == SYS_UI_VIEW_TIME)
  {
    const char *history[SYS_UI_MAX_RENTAL_HISTORY];
    for (int i = 0; i < ctx->rental_history_count; ++i)
    {
      history[i] = ctx->rentalHistory[i];
    }
    bsp_display_drawTimeHistoryOverlay(&ctx->display, history, ctx->rental_history_count, ctx->remaining_minutes,
                                       ctx->remaining_seconds);
  }
  else
  {
    ctx->pending_main_redraw = true;
  }
}

static void sys_ui_updateEnvironmentTelemetry(sys_ui_t *ctx)
{
  // NOTE: Randomized environment telemetry until temperature/humidity sensors are online.
  ctx->temperature_C += static_cast<float>(random(-10, 11)) / 10.0f;
  if (ctx->temperature_C < 20.0f)
    ctx->temperature_C = 20.0f;
  if (ctx->temperature_C > 40.0f)
    ctx->temperature_C = 40.0f;

  ctx->humidity += static_cast<float>(random(-20, 21)) / 10.0f;
  if (ctx->humidity < 40.0f)
    ctx->humidity = 40.0f;
  if (ctx->humidity > 95.0f)
    ctx->humidity = 95.0f;

  ctx->air_quality += random(-5, 6);
  if (ctx->air_quality < 30)
    ctx->air_quality = 30;
  if (ctx->air_quality > 100)
    ctx->air_quality = 100;

  sys_ui_logTemperatureSample(ctx, ctx->temperature_C, OS_GET_TICK());

  if (ctx->view == SYS_UI_VIEW_MAIN)
  {
    bsp_display_updateEnvironment(&ctx->display, ctx->temperature_C, ctx->humidity, ctx->air_quality);
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

static void sys_ui_enterView(sys_ui_t *ctx, sys_ui_view_t view)
{
  ctx->view = view;
  switch (view)
  {
  case SYS_UI_VIEW_SETTINGS:
    bsp_display_drawSettingsOverlay(&ctx->display, ctx->brightness_percent, ctx->theme_background_color);
    /* Register settings screen callbacks */
    if (ctx->display.settings_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.settings_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->display.brightness_slider != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.brightness_slider, sys_ui_brightness_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    }
    for (int i = 0; i < 3; ++i)
    {
      if (ctx->display.color_btns[i] != nullptr)
      {
        lv_obj_add_event_cb(ctx->display.color_btns[i], sys_ui_color_btn_cb, LV_EVENT_CLICKED, (void *) (intptr_t) i);
      }
    }
    break;
  case SYS_UI_VIEW_OUT:
    bsp_display_drawOutOverlay(&ctx->display);
    /* Register out screen back button callback */
    if (ctx->display.out_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.out_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  case SYS_UI_VIEW_TIME:
  {
    const char *history[SYS_UI_MAX_RENTAL_HISTORY];
    for (int i = 0; i < ctx->rental_history_count; ++i)
    {
      history[i] = ctx->rentalHistory[i];
    }
    bsp_display_drawTimeHistoryOverlay(&ctx->display, history, ctx->rental_history_count, ctx->remaining_minutes,
                                       ctx->remaining_seconds);
    /* Register time history screen callbacks */
    if (ctx->display.time_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.time_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->display.extend_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.extend_btn, sys_ui_extend_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    uint32_t now   = OS_GET_TICK();
    float    hours = (now - ctx->session_start_ms) / 3600000.0f;
    if (hours < 0.001f)
      hours = 0.001f;
    float avgSpeed = ctx->distance_km / hours;
    bsp_display_drawDistanceOverlay(&ctx->display, avgSpeed, ctx->distance_km, ctx->distanceHistory,
                                    ctx->distance_history_count);
    /* Register distance screen back button callback */
    if (ctx->display.distance_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.distance_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
    sys_ui_refreshTemperatureOverlay(ctx);
    /* Register temperature screen callbacks */
    if (ctx->display.temp_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.temp_back_btn, sys_ui_back_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->display.zoom_minus_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.zoom_minus_btn, sys_ui_zoom_minus_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->display.zoom_plus_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.zoom_plus_btn, sys_ui_zoom_plus_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->display.pan_left_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.pan_left_btn, sys_ui_pan_left_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    if (ctx->display.pan_right_btn != nullptr)
    {
      lv_obj_add_event_cb(ctx->display.pan_right_btn, sys_ui_pan_right_btn_cb, LV_EVENT_CLICKED, nullptr);
    }
    break;
  default: break;
  }
}

static void sys_ui_returnToMain(sys_ui_t *ctx)
{
  ctx->view                = SYS_UI_VIEW_MAIN;
  ctx->pending_main_redraw = false;
  bsp_display_draw_full_ui(&ctx->display, ctx->remaining_minutes, ctx->remaining_seconds, ctx->distance_km,
                           ctx->current_speed, ctx->temperature_C, ctx->humidity, ctx->air_quality);
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

static void sys_ui_logTemperatureSample(sys_ui_t *ctx, float value, uint32_t timestamp)
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
  if (ctx->temperature_sample_count < 2)
  {
    bsp_display_drawTemperatureOverlay(&ctx->display, ctx->temperatureHistory, ctx->temperatureTimestamps,
                                       ctx->temperature_sample_count, ctx->temperature_zoom, ctx->temperature_pan);
    return;
  }

  if (ctx->temperature_pan > ctx->temperature_sample_count - 2)
  {
    ctx->temperature_pan = ctx->temperature_sample_count - 2;
  }
  if (ctx->temperature_pan < 0)
  {
    ctx->temperature_pan = 0;
  }

  bsp_display_drawTemperatureOverlay(&ctx->display, ctx->temperatureHistory, ctx->temperatureTimestamps,
                                     ctx->temperature_sample_count, ctx->temperature_zoom, ctx->temperature_pan);
}

/**
 * LVGL Touch Input Callback
 * Called by LVGL driver to read touch events
 */
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  /* Get context from driver user data */
  sys_ui_t *ctx = (sys_ui_t *) lv_indev_get_user_data(indev);
  if (ctx == nullptr)
  {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  /* Read touch using existing BSP API */
  bsp_touch_point_t point = { 0 };
  bsp_touch_read(&ctx->touch, &point);

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

/* ============== LVGL Button Event Callbacks ============== */

static void sys_ui_settings_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Settings button pressed\n");
    sys_ui_enterView(g_ui_ctx, SYS_UI_VIEW_SETTINGS);
  }
}

static void sys_ui_out_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Out button pressed\n");
    sys_ui_enterView(g_ui_ctx, SYS_UI_VIEW_OUT);
  }
}

static void sys_ui_back_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Back button pressed\n");
    sys_ui_returnToMain(g_ui_ctx);
  }
}

static void sys_ui_extend_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Extend button pressed\n");
    g_ui_ctx->remaining_minutes += 30;
    const char *history[SYS_UI_MAX_RENTAL_HISTORY];
    for (int i = 0; i < g_ui_ctx->rental_history_count; ++i)
    {
      history[i] = g_ui_ctx->rentalHistory[i];
    }
    bsp_display_drawTimeHistoryOverlay(&g_ui_ctx->display, history, g_ui_ctx->rental_history_count,
                                       g_ui_ctx->remaining_minutes, g_ui_ctx->remaining_seconds);
  }
}

static void sys_ui_brightness_slider_cb(lv_event_t *e)
{
  if (g_ui_ctx == nullptr)
    return;

  lv_obj_t *slider             = (lv_obj_t *) lv_event_get_target(e);
  g_ui_ctx->brightness_percent = lv_slider_get_value(slider);
  bsp_display_set_brightness_percent(&g_ui_ctx->display, g_ui_ctx->brightness_percent);
  printf("[LVGL] Brightness changed to %d%%\n", g_ui_ctx->brightness_percent);
}

static void sys_ui_color_btn_cb(lv_event_t *e)
{
  if (g_ui_ctx == nullptr)
    return;

  lv_obj_t *btn       = (lv_obj_t *) lv_event_get_target(e);
  int       btn_index = (int) (intptr_t) lv_event_get_user_data(e);

  uint16_t colors[3] = { BSP_DISPLAY_HEX_TO_565(TFT_BLACK), BSP_DISPLAY_HEX_TO_565(TFT_WHITE),
                         BSP_DISPLAY_HEX_TO_565(TFT_LIGHTGREY) };
  if (btn_index >= 0 && btn_index < 3)
  {
    g_ui_ctx->theme_background_color = colors[btn_index];
    bsp_display_set_background_color(&g_ui_ctx->display, g_ui_ctx->theme_background_color);
    g_ui_ctx->pending_main_redraw = true;
    printf("[LVGL] Background color changed to index %d\n", btn_index);
  }
  (void) btn;
}

static void sys_ui_zoom_minus_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr && g_ui_ctx->temperature_zoom > 1)
  {
    --g_ui_ctx->temperature_zoom;
    sys_ui_refreshTemperatureOverlay(g_ui_ctx);
    printf("[LVGL] Zoom- pressed, zoom=%d\n", g_ui_ctx->temperature_zoom);
  }
}

static void sys_ui_zoom_plus_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr && g_ui_ctx->temperature_zoom < 4)
  {
    ++g_ui_ctx->temperature_zoom;
    sys_ui_refreshTemperatureOverlay(g_ui_ctx);
    printf("[LVGL] Zoom+ pressed, zoom=%d\n", g_ui_ctx->temperature_zoom);
  }
}

static void sys_ui_pan_left_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    g_ui_ctx->temperature_pan -= 5;
    if (g_ui_ctx->temperature_pan < 0)
      g_ui_ctx->temperature_pan = 0;
    sys_ui_refreshTemperatureOverlay(g_ui_ctx);
    printf("[LVGL] Pan left pressed, pan=%d\n", g_ui_ctx->temperature_pan);
  }
}

static void sys_ui_pan_right_btn_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    g_ui_ctx->temperature_pan += 5;
    if (g_ui_ctx->temperature_pan > g_ui_ctx->temperature_sample_count - 2)
      g_ui_ctx->temperature_pan = g_ui_ctx->temperature_sample_count - 2;
    if (g_ui_ctx->temperature_pan < 0)
      g_ui_ctx->temperature_pan = 0;
    sys_ui_refreshTemperatureOverlay(g_ui_ctx);
    printf("[LVGL] Pan right pressed, pan=%d\n", g_ui_ctx->temperature_pan);
  }
}

static void sys_ui_time_card_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Time card pressed\n");
    sys_ui_enterView(g_ui_ctx, SYS_UI_VIEW_TIME);
  }
}

static void sys_ui_distance_card_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Distance card pressed\n");
    sys_ui_enterView(g_ui_ctx, SYS_UI_VIEW_DISTANCE);
  }
}

static void sys_ui_env_card_cb(lv_event_t *e)
{
  (void) e;
  if (g_ui_ctx != nullptr)
  {
    printf("[LVGL] Environment card pressed\n");
    sys_ui_enterView(g_ui_ctx, SYS_UI_VIEW_TEMPERATURE);
  }
}

/* Register LVGL event callbacks for all interactive widgets */
static void sys_ui_register_lvgl_callbacks(sys_ui_t *ctx)
{
  if (ctx == nullptr)
    return;

  /* Main screen buttons */
  if (ctx->display.settings_btn != nullptr)
  {
    lv_obj_add_event_cb(ctx->display.settings_btn, sys_ui_settings_btn_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (ctx->display.out_btn != nullptr)
  {
    lv_obj_add_event_cb(ctx->display.out_btn, sys_ui_out_btn_cb, LV_EVENT_CLICKED, nullptr);
  }

  /* Main screen cards (clickable) */
  if (ctx->display.time_card != nullptr)
  {
    lv_obj_add_flag(ctx->display.time_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ctx->display.time_card, sys_ui_time_card_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (ctx->display.distance_card != nullptr)
  {
    lv_obj_add_flag(ctx->display.distance_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ctx->display.distance_card, sys_ui_distance_card_cb, LV_EVENT_CLICKED, nullptr);
  }
  if (ctx->display.env_card != nullptr)
  {
    lv_obj_add_flag(ctx->display.env_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ctx->display.env_card, sys_ui_env_card_cb, LV_EVENT_CLICKED, nullptr);
  }
}

/* End of file -------------------------------------------------------- */
