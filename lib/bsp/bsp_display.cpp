/**
 * @file       bsp_display.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0 (LVGL Full Integration)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      Implementation of the board support display using LVGL widgets.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"

#include <SPI.h>
#include <math.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
namespace
{
constexpr int BSP_DISPLAY_SPEEDO_CX      = 90;
constexpr int BSP_DISPLAY_SPEEDO_CY      = 150;
constexpr int BSP_DISPLAY_SPEEDO_OUTER_R = 70;
constexpr int BSP_DISPLAY_SPEEDO_INNER_R = 50;

#if defined(ESP32) && defined(TFT_BL)
constexpr int BACKLIGHT_PWM_CH   = 7;
constexpr int BACKLIGHT_PWM_FREQ = 5000;
constexpr int BACKLIGHT_PWM_BITS = 8;
#endif
}  // namespace

/* Private function prototypes ---------------------------------------- */
static void      bsp_display_createMainScreen(bsp_display_info_t *ctx);
static void      bsp_display_createSettingsScreen(bsp_display_info_t *ctx);
static void      bsp_display_createOutScreen(bsp_display_info_t *ctx);
static void      bsp_display_createTimeHistoryScreen(bsp_display_info_t *ctx);
static void      bsp_display_createDistanceScreen(bsp_display_info_t *ctx);
static void      bsp_display_createTemperatureScreen(bsp_display_info_t *ctx);
static void      bsp_display_ensureBacklightChannel(bsp_display_info_t *ctx);
static lv_obj_t *bsp_display_createCard(lv_obj_t *parent, int x, int y, int w, int h, uint32_t borderColor);
static lv_obj_t *bsp_display_createButton(lv_obj_t   *parent,
                                          int         x,
                                          int         y,
                                          int         w,
                                          int         h,
                                          const char *label,
                                          uint32_t    bgColor,
                                          uint32_t    textColor);
static void      bsp_display_showScreen(bsp_display_info_t *ctx, lv_obj_t *screen);

/* Function definitions ----------------------------------------------- */
void bsp_display_initContext(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  // Initialize default state values
  ctx->prev_speed_int     = -1;
  ctx->battery_percent    = 85;
  ctx->background_color   = BSP_DISPLAY_COLOR_BG;
  ctx->brightness_percent = 80;
  ctx->backlight_en       = false;
  ctx->current_screen     = BSP_DISPLAY_SCREEN_MAIN;
  ctx->active_screen      = nullptr;

  /* Initialize all widget pointers to nullptr */
  ctx->main_screen         = nullptr;
  ctx->header_panel        = nullptr;
  ctx->header_title        = nullptr;
  ctx->header_subtitle     = nullptr;
  ctx->status_led          = nullptr;
  ctx->battery_bar         = nullptr;
  ctx->speedometer_arc     = nullptr;
  ctx->speed_label         = nullptr;
  ctx->speed_unit_label    = nullptr;
  ctx->time_card           = nullptr;
  ctx->time_label          = nullptr;
  ctx->time_unit_label     = nullptr;
  ctx->distance_card       = nullptr;
  ctx->distance_label      = nullptr;
  ctx->distance_unit_label = nullptr;
  ctx->env_card            = nullptr;
  ctx->temp_label          = nullptr;
  ctx->humidity_label      = nullptr;
  ctx->aqi_label           = nullptr;
  ctx->settings_btn        = nullptr;
  ctx->out_btn             = nullptr;
  ctx->settings_screen     = nullptr;
  ctx->out_screen          = nullptr;
  ctx->time_history_screen = nullptr;
  ctx->distance_screen     = nullptr;
  ctx->temp_screen         = nullptr;
  ctx->back_btn            = nullptr;
}

void bsp_display_initialize(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  ctx->tft.init();
  ctx->tft.setRotation(1);
  ctx->tft.fillScreen(TFT_BLACK);
  ctx->prev_speed_int = -1;
  bsp_display_set_brightness_percent(ctx, ctx->brightness_percent);
}

void bsp_display_show_splash(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  /* Use direct TFT for splash screen (before LVGL is fully ready) */
  ctx->tft.fillScreen(TFT_BLACK);
  ctx->tft.drawRect(20, 40, BSP_DISPLAY_SCREEN_W - 40, 160, TFT_CYAN);

  ctx->tft.setTextSize(3);
  ctx->tft.setTextColor(TFT_CYAN, TFT_BLACK);
  ctx->tft.setCursor(50, 95);
  ctx->tft.print("SMART BIKE");

  ctx->tft.setTextSize(2);
  ctx->tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  ctx->tft.setCursor(105, 125);
  ctx->tft.print("RENTAL");

  ctx->tft.setTextSize(1);
  ctx->tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  ctx->tft.setCursor(125, 150);
  ctx->tft.print("Version 1.0");

  ctx->tft.drawRect(50, 175, BSP_DISPLAY_SCREEN_W - 100, 10, TFT_DARKGREY);
  for (int i = 0; i <= 100; i += 5)
  {
    int w = ((BSP_DISPLAY_SCREEN_W - 104) * i) / 100;
    ctx->tft.fillRect(52, 177, w, 6, TFT_CYAN);
    delay(20);
  }
  delay(300);
}

void bsp_display_set_battery_percent(bsp_display_info_t *ctx, int percent)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;
  ctx->battery_percent = percent;

  /* Update battery bar if exists */
  if (ctx->battery_bar != nullptr)
  {
    lv_bar_set_value(ctx->battery_bar, percent, LV_ANIM_OFF);
  }
}

void bsp_display_draw_full_ui(bsp_display_info_t *ctx,
                              int                 minutes,
                              int                 seconds,
                              float               distance_km,
                              float               current_speed,
                              float               temperature_C,
                              float               humidity,
                              int                 air_quality)
{
  if (ctx == nullptr)
  {
    return;
  }

  /* Create main screen if not exists */
  if (ctx->main_screen == nullptr)
  {
    bsp_display_createMainScreen(ctx);
  }

  /* Show main screen */
  bsp_display_showScreen(ctx, ctx->main_screen);

  /* Update all values */
  bsp_display_updateTime(ctx, minutes, seconds);
  bsp_display_updateDistance(ctx, distance_km);
  bsp_display_updateEnvironment(ctx, temperature_C, humidity, air_quality);
  bsp_display_updateSpeed(ctx, static_cast<int>(current_speed));
}

void bsp_display_updateSpeed(bsp_display_info_t *ctx, int speedKph)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (speedKph < 0)
    speedKph = 0;
  if (speedKph > 40)
    speedKph = 40;

  if (ctx->prev_speed_int == speedKph)
  {
    return;
  }

  ctx->prev_speed_int = speedKph;

  /* Update speedometer arc */
  if (ctx->speedometer_arc != nullptr)
  {
    /* Map speed 0-40 to arc value 0-100 */
    int arcValue = (speedKph * 100) / 40;
    lv_arc_set_value(ctx->speedometer_arc, arcValue);

    /* Change arc color based on speed */
    lv_color_t arcColor;
    if (speedKph <= 20)
    {
      arcColor = lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS);
    }
    else if (speedKph <= 30)
    {
      arcColor = lv_color_hex(BSP_DISPLAY_COLOR_WARNING);
    }
    else
    {
      arcColor = lv_color_hex(BSP_DISPLAY_COLOR_DANGER);
    }
    lv_obj_set_style_arc_color(ctx->speedometer_arc, arcColor, LV_PART_INDICATOR);
  }

  /* Update speed label */
  if (ctx->speed_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->speed_label, "%d", speedKph);
  }
}

void bsp_display_updateTime(bsp_display_info_t *ctx, int minutes, int seconds)
{
  if (ctx == nullptr || ctx->time_label == nullptr)
  {
    return;
  }

  lv_label_set_text_fmt(ctx->time_label, "%02d:%02d", minutes, seconds);

  /* Change color based on remaining time */
  lv_color_t timeColor;
  if (minutes < 5)
  {
    timeColor = lv_color_hex(BSP_DISPLAY_COLOR_DANGER);
  }
  else if (minutes < 10)
  {
    timeColor = lv_color_hex(BSP_DISPLAY_COLOR_WARNING);
  }
  else
  {
    timeColor = lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS);
  }
  lv_obj_set_style_text_color(ctx->time_label, timeColor, 0);
}

void bsp_display_updateDistance(bsp_display_info_t *ctx, float distance_km)
{
  if (ctx == nullptr || ctx->distance_label == nullptr)
  {
    return;
  }

  lv_label_set_text_fmt(ctx->distance_label, "%.2f", distance_km);
}

void bsp_display_updateEnvironment(bsp_display_info_t *ctx, float temperature_C, float humidity, int air_quality)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (ctx->temp_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->temp_label, "%.1f°C", temperature_C);
  }

  if (ctx->humidity_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->humidity_label, "%.0f%%", humidity);
  }

  if (ctx->aqi_label != nullptr)
  {
    const char *status = (air_quality >= 80) ? "Good" : ((air_quality >= 50) ? "Fair" : "Poor");
    lv_label_set_text_fmt(ctx->aqi_label, "AQI:%d %s", air_quality, status);

    lv_color_t aqiColor;
    if (air_quality >= 80)
    {
      aqiColor = lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS);
    }
    else if (air_quality >= 50)
    {
      aqiColor = lv_color_hex(BSP_DISPLAY_COLOR_WARNING);
    }
    else
    {
      aqiColor = lv_color_hex(BSP_DISPLAY_COLOR_DANGER);
    }
    lv_obj_set_style_text_color(ctx->aqi_label, aqiColor, 0);
  }
}

void bsp_display_set_background_color(bsp_display_info_t *ctx, uint16_t color)
{
  if (ctx == nullptr)
  {
    return;
  }

  /* Convert 565 to hex color */
  uint8_t  red   = ((color >> 11) & 0x1F) << 3;
  uint8_t  green = ((color >> 5) & 0x3F) << 2;
  uint8_t  blue  = (color & 0x1F) << 3;
  uint32_t hex   = (red << 16) | (green << 8) | blue;

  ctx->background_color = hex;

  /* Update main screen background if exists */
  if (ctx->main_screen != nullptr)
  {
    lv_obj_set_style_bg_color(ctx->main_screen, lv_color_hex(hex), 0);
  }
}

uint16_t bsp_display_cvt_background_color(const bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return 0;
  }

  /* Convert hex to 565 */
  uint32_t hex   = ctx->background_color;
  uint8_t  red   = (hex >> 16) & 0xFF;
  uint8_t  green = (hex >> 8) & 0xFF;
  uint8_t  blue  = hex & 0xFF;

  /**
   * 565 format: 5 bit red, 6 bit green, 5 bit blue
   * RRRRRGGG GGG BBBBB
   */
  return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3);
}

void bsp_display_set_brightness_percent(bsp_display_info_t *ctx, int percent)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;
  ctx->brightness_percent = percent;

#if defined(ESP32) && defined(TFT_BL)
  bsp_display_ensureBacklightChannel(ctx);
  int max_duty = (1 << BACKLIGHT_PWM_BITS) - 1;
  int duty     = map(ctx->brightness_percent, 0, 100, 0, max_duty);
  ledcWrite(BACKLIGHT_PWM_CH, duty);
#endif

  /* Update brightness slider if exists */
  if (ctx->brightness_slider != nullptr)
  {
    lv_slider_set_value(ctx->brightness_slider, percent, LV_ANIM_OFF);
  }
}

int bsp_display_brightnessPercent(const bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return 0;
  }

  return ctx->brightness_percent;
}

void bsp_display_drawSettingsOverlay(bsp_display_info_t *ctx, int brightness_percent, uint16_t bgColor)
{
  if (ctx == nullptr)
  {
    return;
  }

  /* Create settings screen if not exists */
  if (ctx->settings_screen == nullptr)
  {
    bsp_display_createSettingsScreen(ctx);
  }

  /* Update brightness value */
  if (ctx->brightness_slider != nullptr)
  {
    lv_slider_set_value(ctx->brightness_slider, brightness_percent, LV_ANIM_OFF);
  }
  if (ctx->brightness_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->brightness_label, "%d%%", brightness_percent);
  }

  bsp_display_showScreen(ctx, ctx->settings_screen);
}

void bsp_display_drawOutOverlay(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (ctx->out_screen == nullptr)
  {
    bsp_display_createOutScreen(ctx);
  }

  bsp_display_showScreen(ctx, ctx->out_screen);
}

void bsp_display_drawTimeHistoryOverlay(bsp_display_info_t *ctx,
                                        const char         *history[],
                                        int                 count,
                                        int                 remaining_minutes,
                                        int                 remaining_seconds)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (ctx->time_history_screen == nullptr)
  {
    bsp_display_createTimeHistoryScreen(ctx);
  }

  /* Update history labels */
  for (int i = 0; i < 4; ++i)
  {
    if (ctx->history_labels[i] != nullptr)
    {
      if (i < count && history[i] != nullptr)
      {
        lv_label_set_text(ctx->history_labels[i], history[i]);
      }
      else
      {
        lv_label_set_text(ctx->history_labels[i], "");
      }
    }
  }

  /* Update remaining time */
  if (ctx->time_remaining_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->time_remaining_label, "Remaining: %02d:%02d", remaining_minutes, remaining_seconds);
  }

  bsp_display_showScreen(ctx, ctx->time_history_screen);
}

void bsp_display_drawDistanceOverlay(bsp_display_info_t *ctx,
                                     float               avgSpeed,
                                     float               totalKm,
                                     const float        *history,
                                     int                 count)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (ctx->distance_screen == nullptr)
  {
    bsp_display_createDistanceScreen(ctx);
  }

  /* Update labels */
  if (ctx->total_distance_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->total_distance_label, "Total: %.2f km", totalKm);
  }

  if (ctx->avg_speed_label != nullptr)
  {
    lv_label_set_text_fmt(ctx->avg_speed_label, "Avg speed: %.1f km/h", avgSpeed);
  }

  /* Update chart data */
  if (ctx->distance_chart != nullptr && ctx->distance_series != nullptr)
  {
    lv_chart_set_point_count(ctx->distance_chart, count > 0 ? count : 1);
    for (int i = 0; i < count && i < 16; ++i)
    {
      lv_chart_set_value_by_id(ctx->distance_chart, ctx->distance_series, i, (int) (history[i] * 100));
    }
    lv_chart_refresh(ctx->distance_chart);
  }

  bsp_display_showScreen(ctx, ctx->distance_screen);
}

void bsp_display_drawTemperatureOverlay(bsp_display_info_t *ctx,
                                        const float        *samples,
                                        const uint32_t     *timestamps,
                                        int                 count,
                                        int                 zoomLevel,
                                        int                 panOffset)
{
  if (ctx == nullptr)
  {
    return;
  }

  if (ctx->temp_screen == nullptr)
  {
    bsp_display_createTemperatureScreen(ctx);
  }

  /* Update chart with visible samples */
  if (ctx->temp_chart != nullptr && ctx->temp_series != nullptr && count > 1)
  {
    int window = count / zoomLevel;
    if (window < 2)
      window = 2;
    if (window > count)
      window = count;
    if (panOffset < 0)
      panOffset = 0;
    if (panOffset > count - window)
      panOffset = count - window;

    /* Find min/max for scaling */
    float minTemp = samples[panOffset];
    float maxTemp = samples[panOffset];
    for (int i = 0; i < window; ++i)
    {
      float v = samples[panOffset + i];
      if (v < minTemp)
        minTemp = v;
      if (v > maxTemp)
        maxTemp = v;
    }

    /* Update range label */
    if (ctx->temp_range_label != nullptr)
    {
      float span = 0;
      if (count > 0)
      {
        span = (timestamps[panOffset + window - 1] - timestamps[panOffset]) / 1000.0f;
      }
      lv_label_set_text_fmt(ctx->temp_range_label, "%.1f-%.1fC (%.1fs)", minTemp, maxTemp, span);
    }

    /* Update chart */
    lv_chart_set_point_count(ctx->temp_chart, window);
    lv_chart_set_range(ctx->temp_chart, LV_CHART_AXIS_PRIMARY_Y, (int) (minTemp * 10) - 5, (int) (maxTemp * 10) + 5);

    for (int i = 0; i < window; ++i)
    {
      lv_chart_set_value_by_id(ctx->temp_chart, ctx->temp_series, i, (int) (samples[panOffset + i] * 10));
    }
    lv_chart_refresh(ctx->temp_chart);
  }

  bsp_display_showScreen(ctx, ctx->temp_screen);
}

TFT_eSPI *bsp_display_driver(bsp_display_info_t *ctx)
{
  if (ctx == nullptr)
  {
    return nullptr;
  }

  return &ctx->tft;
}

/* Private definitions ----------------------------------------------- */
static void bsp_display_showScreen(bsp_display_info_t *ctx, lv_obj_t *screen)
{
  if (ctx == nullptr || screen == nullptr)
  {
    return;
  }

  ctx->active_screen = screen;
  lv_screen_load(screen);
}

static lv_obj_t *bsp_display_createCard(lv_obj_t *parent, int x, int y, int w, int h, uint32_t borderColor)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_size(card, w, h);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x18E318), 0);
  lv_obj_set_style_border_color(card, lv_color_hex(borderColor), 0);
  lv_obj_set_style_border_width(card, 1, 0);
  lv_obj_set_style_radius(card, 4, 0);
  lv_obj_set_style_pad_all(card, 4, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  return card;
}

static lv_obj_t *bsp_display_createButton(lv_obj_t   *parent,
                                          int         x,
                                          int         y,
                                          int         w,
                                          int         h,
                                          const char *label,
                                          uint32_t    bgColor,
                                          uint32_t    textColor)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_style_bg_color(btn, lv_color_hex(bgColor), 0);
  lv_obj_set_style_radius(btn, 6, 0);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, label);
  lv_obj_set_style_text_color(lbl, lv_color_hex(textColor), 0);
  lv_obj_center(lbl);

  return btn;
}

static void bsp_display_createMainScreen(bsp_display_info_t *ctx)
{
  /* Create main screen */
  ctx->main_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ctx->main_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(ctx->main_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Create header panel */
  ctx->header_panel = lv_obj_create(ctx->main_screen);
  lv_obj_set_pos(ctx->header_panel, 0, 0);
  lv_obj_set_size(ctx->header_panel, BSP_DISPLAY_SCREEN_W, 26);
  lv_obj_set_style_bg_color(ctx->header_panel, lv_color_hex(0x002840), 0);
  lv_obj_set_style_border_width(ctx->header_panel, 0, 0);
  lv_obj_set_style_radius(ctx->header_panel, 0, 0);
  lv_obj_set_style_pad_all(ctx->header_panel, 0, 0);
  lv_obj_remove_flag(ctx->header_panel, LV_OBJ_FLAG_SCROLLABLE);

  /* Header title */
  ctx->header_title = lv_label_create(ctx->header_panel);
  lv_label_set_text(ctx->header_title, "SMART BIKE");
  lv_obj_set_pos(ctx->header_title, 35, 2);
  lv_obj_set_style_text_color(ctx->header_title, lv_color_hex(BSP_DISPLAY_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(ctx->header_title, &lv_font_montserrat_10, 0);

  /* Header subtitle */
  ctx->header_subtitle = lv_label_create(ctx->header_panel);
  lv_label_set_text(ctx->header_subtitle, "RENTAL #B-1024");
  lv_obj_set_pos(ctx->header_subtitle, 35, 13);
  lv_obj_set_style_text_color(ctx->header_subtitle, lv_color_hex(BSP_DISPLAY_COLOR_ACCENT), 0);
  lv_obj_set_style_text_font(ctx->header_subtitle, &lv_font_montserrat_10, 0);

  /* Status LED */
  ctx->status_led = lv_obj_create(ctx->header_panel);
  lv_obj_set_pos(ctx->status_led, BSP_DISPLAY_SCREEN_W - 55, 9);
  lv_obj_set_size(ctx->status_led, 8, 8);
  lv_obj_set_style_bg_color(ctx->status_led, lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS), 0);
  lv_obj_set_style_radius(ctx->status_led, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(ctx->status_led, 0, 0);

  /* Battery bar */
  ctx->battery_bar = lv_bar_create(ctx->header_panel);
  lv_obj_set_pos(ctx->battery_bar, BSP_DISPLAY_SCREEN_W - 30, 8);
  lv_obj_set_size(ctx->battery_bar, 20, 10);
  lv_bar_set_range(ctx->battery_bar, 0, 100);
  lv_bar_set_value(ctx->battery_bar, ctx->battery_percent, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(ctx->battery_bar, lv_color_hex(0x333333), 0);
  lv_obj_set_style_bg_color(ctx->battery_bar, lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS), LV_PART_INDICATOR);

  /* Speedometer Arc */
  ctx->speedometer_arc = lv_arc_create(ctx->main_screen);
  lv_obj_set_pos(ctx->speedometer_arc, BSP_DISPLAY_SPEEDO_CX - BSP_DISPLAY_SPEEDO_OUTER_R,
                 BSP_DISPLAY_SPEEDO_CY - BSP_DISPLAY_SPEEDO_OUTER_R);
  lv_obj_set_size(ctx->speedometer_arc, BSP_DISPLAY_SPEEDO_OUTER_R * 2, BSP_DISPLAY_SPEEDO_OUTER_R * 2);
  lv_arc_set_rotation(ctx->speedometer_arc, 135);
  lv_arc_set_bg_angles(ctx->speedometer_arc, 0, 270);
  lv_arc_set_range(ctx->speedometer_arc, 0, 100);
  lv_arc_set_value(ctx->speedometer_arc, 0);
  lv_obj_set_style_arc_width(ctx->speedometer_arc, 10, LV_PART_MAIN);
  lv_obj_set_style_arc_width(ctx->speedometer_arc, 10, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(ctx->speedometer_arc, lv_color_hex(0x333333), LV_PART_MAIN);
  lv_obj_set_style_arc_color(ctx->speedometer_arc, lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS), LV_PART_INDICATOR);
  lv_obj_remove_style(ctx->speedometer_arc, nullptr, LV_PART_KNOB);
  lv_obj_remove_flag(ctx->speedometer_arc, LV_OBJ_FLAG_CLICKABLE);

  /* Speed label (center of arc) */
  ctx->speed_label = lv_label_create(ctx->main_screen);
  lv_label_set_text(ctx->speed_label, "0");
  lv_obj_set_pos(ctx->speed_label, BSP_DISPLAY_SPEEDO_CX - 20, BSP_DISPLAY_SPEEDO_CY - 10);
  lv_obj_set_style_text_color(ctx->speed_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(ctx->speed_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_align(ctx->speed_label, LV_TEXT_ALIGN_CENTER, 0);

  /* Speed unit label */
  ctx->speed_unit_label = lv_label_create(ctx->main_screen);
  lv_label_set_text(ctx->speed_unit_label, "km/h");
  lv_obj_set_pos(ctx->speed_unit_label, BSP_DISPLAY_SPEEDO_CX - 15, BSP_DISPLAY_SPEEDO_CY + 20);
  lv_obj_set_style_text_color(ctx->speed_unit_label, lv_color_hex(BSP_DISPLAY_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(ctx->speed_unit_label, &lv_font_montserrat_10, 0);

  /* Time Card */
  ctx->time_card = bsp_display_createCard(ctx->main_screen, BSP_DISPLAY_TIME_CARD_X, BSP_DISPLAY_TIME_CARD_Y,
                                          BSP_DISPLAY_TIME_CARD_W, BSP_DISPLAY_TIME_CARD_H, BSP_DISPLAY_COLOR_PRIMARY);

  lv_obj_t *time_title = lv_label_create(ctx->time_card);
  lv_label_set_text(time_title, "REMAIN");
  lv_obj_set_pos(time_title, 10, 2);
  lv_obj_set_style_text_color(time_title, lv_color_hex(BSP_DISPLAY_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(time_title, &lv_font_montserrat_10, 0);

  ctx->time_label = lv_label_create(ctx->time_card);
  lv_label_set_text(ctx->time_label, "00:00");
  lv_obj_set_pos(ctx->time_label, 5, 18);
  lv_obj_set_style_text_color(ctx->time_label, lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS), 0);
  lv_obj_set_style_text_font(ctx->time_label, &lv_font_montserrat_18, 0);

  ctx->time_unit_label = lv_label_create(ctx->time_card);
  lv_label_set_text(ctx->time_unit_label, "mins");
  lv_obj_set_pos(ctx->time_unit_label, 25, 38);
  lv_obj_set_style_text_color(ctx->time_unit_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(ctx->time_unit_label, &lv_font_montserrat_10, 0);

  /* Distance Card */
  ctx->distance_card =
    bsp_display_createCard(ctx->main_screen, BSP_DISPLAY_DIST_CARD_X, BSP_DISPLAY_DIST_CARD_Y, BSP_DISPLAY_DIST_CARD_W,
                           BSP_DISPLAY_DIST_CARD_H, BSP_DISPLAY_COLOR_ACCENT);

  lv_obj_t *dist_title = lv_label_create(ctx->distance_card);
  lv_label_set_text(dist_title, "DISTANCE");
  lv_obj_set_pos(dist_title, 5, 2);
  lv_obj_set_style_text_color(dist_title, lv_color_hex(BSP_DISPLAY_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(dist_title, &lv_font_montserrat_10, 0);

  ctx->distance_label = lv_label_create(ctx->distance_card);
  lv_label_set_text(ctx->distance_label, "0.00");
  lv_obj_set_pos(ctx->distance_label, 5, 16);
  lv_obj_set_style_text_color(ctx->distance_label, lv_color_hex(BSP_DISPLAY_COLOR_ACCENT), 0);
  lv_obj_set_style_text_font(ctx->distance_label, &lv_font_montserrat_16, 0);

  ctx->distance_unit_label = lv_label_create(ctx->distance_card);
  lv_label_set_text(ctx->distance_unit_label, "km");
  lv_obj_set_pos(ctx->distance_unit_label, 62, 20);
  lv_obj_set_style_text_color(ctx->distance_unit_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(ctx->distance_unit_label, &lv_font_montserrat_10, 0);

  /* Environment Card */
  ctx->env_card = bsp_display_createCard(ctx->main_screen, BSP_DISPLAY_ENV_CARD_X, BSP_DISPLAY_ENV_CARD_Y,
                                         BSP_DISPLAY_ENV_CARD_W, BSP_DISPLAY_ENV_CARD_H, BSP_DISPLAY_COLOR_TEXT_DIM);

  lv_obj_t *env_title = lv_label_create(ctx->env_card);
  lv_label_set_text(env_title, "ENV STATUS");
  lv_obj_set_pos(env_title, 8, 0);
  lv_obj_set_style_text_color(env_title, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(env_title, &lv_font_montserrat_10, 0);

  ctx->temp_label = lv_label_create(ctx->env_card);
  lv_label_set_text(ctx->temp_label, "0.0°C");
  lv_obj_set_pos(ctx->temp_label, 5, 16);
  lv_obj_set_style_text_color(ctx->temp_label, lv_color_hex(BSP_DISPLAY_COLOR_WARNING), 0);
  lv_obj_set_style_text_font(ctx->temp_label, &lv_font_montserrat_12, 0);

  ctx->humidity_label = lv_label_create(ctx->env_card);
  lv_label_set_text(ctx->humidity_label, "0%");
  lv_obj_set_pos(ctx->humidity_label, 5, 32);
  lv_obj_set_style_text_color(ctx->humidity_label, lv_color_hex(BSP_DISPLAY_COLOR_PRIMARY), 0);
  lv_obj_set_style_text_font(ctx->humidity_label, &lv_font_montserrat_12, 0);

  ctx->aqi_label = lv_label_create(ctx->env_card);
  lv_label_set_text(ctx->aqi_label, "AQI: --");
  lv_obj_set_pos(ctx->aqi_label, 5, 48);
  lv_obj_set_style_text_color(ctx->aqi_label, lv_color_hex(BSP_DISPLAY_COLOR_SUCCESS), 0);
  lv_obj_set_style_text_font(ctx->aqi_label, &lv_font_montserrat_10, 0);

  /* Control Buttons */
  ctx->settings_btn = bsp_display_createButton(ctx->main_screen, BSP_DISPLAY_SETTINGS_BTN_X, BSP_DISPLAY_CTRL_BTN_Y,
                                               BSP_DISPLAY_SETTINGS_BTN_W, BSP_DISPLAY_CTRL_BTN_H, "SETTINGS",
                                               BSP_DISPLAY_COLOR_PRIMARY, BSP_DISPLAY_COLOR_BG);

  ctx->out_btn =
    bsp_display_createButton(ctx->main_screen, BSP_DISPLAY_OUT_BTN_X, BSP_DISPLAY_CTRL_BTN_Y, BSP_DISPLAY_OUT_BTN_W,
                             BSP_DISPLAY_CTRL_BTN_H, "OUT", BSP_DISPLAY_COLOR_WARNING, BSP_DISPLAY_COLOR_BG);
}

static void bsp_display_createSettingsScreen(bsp_display_info_t *ctx)
{
  ctx->settings_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ctx->settings_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(ctx->settings_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Back button */
  ctx->settings_back_btn = bsp_display_createButton(
    ctx->settings_screen, BSP_DISPLAY_BACK_BTN_X, BSP_DISPLAY_BACK_BTN_Y, BSP_DISPLAY_BACK_BTN_W,
    BSP_DISPLAY_BACK_BTN_H, "BACK", BSP_DISPLAY_COLOR_ACCENT, BSP_DISPLAY_COLOR_BG);

  /* Title */
  ctx->settings_title = lv_label_create(ctx->settings_screen);
  lv_label_set_text(ctx->settings_title, "SETTINGS");
  lv_obj_set_pos(ctx->settings_title, 90, 15);
  lv_obj_set_style_text_color(ctx->settings_title, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(ctx->settings_title, &lv_font_montserrat_18, 0);

  /* Brightness label */
  lv_obj_t *bright_label = lv_label_create(ctx->settings_screen);
  lv_label_set_text(bright_label, "Brightness");
  lv_obj_set_pos(bright_label, 40, 55);
  lv_obj_set_style_text_color(bright_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);

  /* Brightness slider */
  ctx->brightness_slider = lv_slider_create(ctx->settings_screen);
  lv_obj_set_pos(ctx->brightness_slider, 40, 80);
  lv_obj_set_size(ctx->brightness_slider, 200, 20);
  lv_slider_set_range(ctx->brightness_slider, 5, 100);
  lv_slider_set_value(ctx->brightness_slider, ctx->brightness_percent, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(ctx->brightness_slider, lv_color_hex(0x333333), LV_PART_MAIN);
  lv_obj_set_style_bg_color(ctx->brightness_slider, lv_color_hex(BSP_DISPLAY_COLOR_PRIMARY), LV_PART_INDICATOR);

  /* Brightness percentage label */
  ctx->brightness_label = lv_label_create(ctx->settings_screen);
  lv_label_set_text_fmt(ctx->brightness_label, "%d%%", ctx->brightness_percent);
  lv_obj_set_pos(ctx->brightness_label, 250, 80);
  lv_obj_set_style_text_color(ctx->brightness_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);

  /* Background color label */
  lv_obj_t *bg_label = lv_label_create(ctx->settings_screen);
  lv_label_set_text(bg_label, "Background");
  lv_obj_set_pos(bg_label, 40, 120);
  lv_obj_set_style_text_color(bg_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);

  /* Color selection buttons */
  uint32_t colors[3] = { 0x000000, 0xFFFFFF, 0x7B7B7B };
  for (int i = 0; i < 3; ++i)
  {
    ctx->color_btns[i] = lv_btn_create(ctx->settings_screen);
    lv_obj_set_pos(ctx->color_btns[i], 40 + i * BSP_DISPLAY_SETTINGS_COLOR_SPAN, BSP_DISPLAY_SETTINGS_COLOR_ROW_Y);
    lv_obj_set_size(ctx->color_btns[i], BSP_DISPLAY_SETTINGS_COLOR_SIZE, BSP_DISPLAY_SETTINGS_COLOR_SIZE);
    lv_obj_set_style_bg_color(ctx->color_btns[i], lv_color_hex(colors[i]), 0);
    lv_obj_set_style_radius(ctx->color_btns[i], 4, 0);
  }
}

static void bsp_display_createOutScreen(bsp_display_info_t *ctx)
{
  ctx->out_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ctx->out_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(ctx->out_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Back button */
  ctx->out_back_btn =
    bsp_display_createButton(ctx->out_screen, BSP_DISPLAY_BACK_BTN_X, BSP_DISPLAY_BACK_BTN_Y, BSP_DISPLAY_BACK_BTN_W,
                             BSP_DISPLAY_BACK_BTN_H, "BACK", BSP_DISPLAY_COLOR_ACCENT, BSP_DISPLAY_COLOR_BG);

  /* Title */
  ctx->out_title = lv_label_create(ctx->out_screen);
  lv_label_set_text(ctx->out_title, "OUT MODE");
  lv_obj_set_pos(ctx->out_title, 80, 80);
  lv_obj_set_style_text_color(ctx->out_title, lv_color_hex(BSP_DISPLAY_COLOR_WARNING), 0);
  lv_obj_set_style_text_font(ctx->out_title, &lv_font_montserrat_24, 0);

  /* Instruction */
  lv_obj_t *instr = lv_label_create(ctx->out_screen);
  lv_label_set_text(instr, "Tap BACK to resume.");
  lv_obj_set_pos(instr, 80, 130);
  lv_obj_set_style_text_color(instr, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
}

static void bsp_display_createTimeHistoryScreen(bsp_display_info_t *ctx)
{
  ctx->time_history_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ctx->time_history_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(ctx->time_history_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Back button */
  ctx->time_back_btn = bsp_display_createButton(ctx->time_history_screen, BSP_DISPLAY_BACK_BTN_X,
                                                BSP_DISPLAY_BACK_BTN_Y, BSP_DISPLAY_BACK_BTN_W, BSP_DISPLAY_BACK_BTN_H,
                                                "BACK", BSP_DISPLAY_COLOR_ACCENT, BSP_DISPLAY_COLOR_BG);

  /* Title */
  ctx->time_history_title = lv_label_create(ctx->time_history_screen);
  lv_label_set_text(ctx->time_history_title, "RENTAL HISTORY");
  lv_obj_set_pos(ctx->time_history_title, 60, 15);
  lv_obj_set_style_text_color(ctx->time_history_title, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(ctx->time_history_title, &lv_font_montserrat_18, 0);

  /* History labels */
  for (int i = 0; i < 4; ++i)
  {
    ctx->history_labels[i] = lv_label_create(ctx->time_history_screen);
    lv_label_set_text(ctx->history_labels[i], "");
    lv_obj_set_pos(ctx->history_labels[i], 40, 55 + i * 20);
    lv_obj_set_style_text_color(ctx->history_labels[i], lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
    lv_obj_set_style_text_font(ctx->history_labels[i], &lv_font_montserrat_12, 0);
  }

  /* Remaining time label */
  ctx->time_remaining_label = lv_label_create(ctx->time_history_screen);
  lv_label_set_text(ctx->time_remaining_label, "Remaining: 00:00");
  lv_obj_set_pos(ctx->time_remaining_label, 40, 140);
  lv_obj_set_style_text_color(ctx->time_remaining_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);

  /* Extend button */
  ctx->extend_btn = bsp_display_createButton(
    ctx->time_history_screen, BSP_DISPLAY_EXTEND_BTN_X, BSP_DISPLAY_EXTEND_BTN_Y, BSP_DISPLAY_EXTEND_BTN_W,
    BSP_DISPLAY_EXTEND_BTN_H, "+30 min", BSP_DISPLAY_COLOR_SUCCESS, BSP_DISPLAY_COLOR_BG);
}

static void bsp_display_createDistanceScreen(bsp_display_info_t *ctx)
{
  ctx->distance_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ctx->distance_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(ctx->distance_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Back button */
  ctx->distance_back_btn = bsp_display_createButton(
    ctx->distance_screen, BSP_DISPLAY_BACK_BTN_X, BSP_DISPLAY_BACK_BTN_Y, BSP_DISPLAY_BACK_BTN_W,
    BSP_DISPLAY_BACK_BTN_H, "BACK", BSP_DISPLAY_COLOR_ACCENT, BSP_DISPLAY_COLOR_BG);

  /* Title */
  ctx->distance_title = lv_label_create(ctx->distance_screen);
  lv_label_set_text(ctx->distance_title, "DISTANCE STATS");
  lv_obj_set_pos(ctx->distance_title, 70, 15);
  lv_obj_set_style_text_color(ctx->distance_title, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(ctx->distance_title, &lv_font_montserrat_18, 0);

  /* Total distance label */
  ctx->total_distance_label = lv_label_create(ctx->distance_screen);
  lv_label_set_text(ctx->total_distance_label, "Total: 0.00 km");
  lv_obj_set_pos(ctx->total_distance_label, 30, 50);
  lv_obj_set_style_text_color(ctx->total_distance_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);

  /* Average speed label */
  ctx->avg_speed_label = lv_label_create(ctx->distance_screen);
  lv_label_set_text(ctx->avg_speed_label, "Avg speed: 0.0 km/h");
  lv_obj_set_pos(ctx->avg_speed_label, 30, 70);
  lv_obj_set_style_text_color(ctx->avg_speed_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);

  /* Chart */
  ctx->distance_chart = lv_chart_create(ctx->distance_screen);
  lv_obj_set_pos(ctx->distance_chart, 30, 95);
  lv_obj_set_size(ctx->distance_chart, 260, 120);
  lv_chart_set_type(ctx->distance_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(ctx->distance_chart, 16);
  lv_obj_set_style_bg_color(ctx->distance_chart, lv_color_hex(0x111111), 0);

  ctx->distance_series =
    lv_chart_add_series(ctx->distance_chart, lv_color_hex(BSP_DISPLAY_COLOR_ACCENT), LV_CHART_AXIS_PRIMARY_Y);
}

static void bsp_display_createTemperatureScreen(bsp_display_info_t *ctx)
{
  ctx->temp_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ctx->temp_screen, lv_color_hex(ctx->background_color), 0);
  lv_obj_remove_flag(ctx->temp_screen, LV_OBJ_FLAG_SCROLLABLE);

  /* Back button */
  ctx->temp_back_btn =
    bsp_display_createButton(ctx->temp_screen, BSP_DISPLAY_BACK_BTN_X, BSP_DISPLAY_BACK_BTN_Y, BSP_DISPLAY_BACK_BTN_W,
                             BSP_DISPLAY_BACK_BTN_H, "BACK", BSP_DISPLAY_COLOR_ACCENT, BSP_DISPLAY_COLOR_BG);

  /* Title */
  ctx->temp_title = lv_label_create(ctx->temp_screen);
  lv_label_set_text(ctx->temp_title, "TEMPERATURE");
  lv_obj_set_pos(ctx->temp_title, 90, 15);
  lv_obj_set_style_text_color(ctx->temp_title, lv_color_hex(BSP_DISPLAY_COLOR_TEXT), 0);
  lv_obj_set_style_text_font(ctx->temp_title, &lv_font_montserrat_18, 0);

  /* Chart */
  ctx->temp_chart = lv_chart_create(ctx->temp_screen);
  lv_obj_set_pos(ctx->temp_chart, BSP_DISPLAY_TEMP_GRAPH_X, BSP_DISPLAY_TEMP_GRAPH_Y);
  lv_obj_set_size(ctx->temp_chart, BSP_DISPLAY_TEMP_GRAPH_W, BSP_DISPLAY_TEMP_GRAPH_H);
  lv_chart_set_type(ctx->temp_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(ctx->temp_chart, 60);
  lv_obj_set_style_bg_color(ctx->temp_chart, lv_color_hex(0x111111), 0);

  ctx->temp_series =
    lv_chart_add_series(ctx->temp_chart, lv_color_hex(BSP_DISPLAY_COLOR_ACCENT), LV_CHART_AXIS_PRIMARY_Y);

  /* Range label */
  ctx->temp_range_label = lv_label_create(ctx->temp_screen);
  lv_label_set_text(ctx->temp_range_label, "Collecting...");
  lv_obj_set_pos(ctx->temp_range_label, BSP_DISPLAY_TEMP_GRAPH_X, BSP_DISPLAY_TEMP_GRAPH_Y - 15);
  lv_obj_set_style_text_color(ctx->temp_range_label, lv_color_hex(BSP_DISPLAY_COLOR_TEXT_DIM), 0);
  lv_obj_set_style_text_font(ctx->temp_range_label, &lv_font_montserrat_10, 0);

  /* Zoom/Pan buttons */
  ctx->zoom_minus_btn =
    bsp_display_createButton(ctx->temp_screen, BSP_DISPLAY_TEMP_GRAPH_X, BSP_DISPLAY_TEMP_BTN_Y, BSP_DISPLAY_TEMP_BTN_W,
                             BSP_DISPLAY_TEMP_BTN_H, "Zoom-", 0x333333, BSP_DISPLAY_COLOR_TEXT);

  ctx->zoom_plus_btn = bsp_display_createButton(
    ctx->temp_screen, BSP_DISPLAY_TEMP_GRAPH_X + BSP_DISPLAY_TEMP_BTN_W + BSP_DISPLAY_TEMP_BTN_GAP,
    BSP_DISPLAY_TEMP_BTN_Y, BSP_DISPLAY_TEMP_BTN_W, BSP_DISPLAY_TEMP_BTN_H, "Zoom+", BSP_DISPLAY_COLOR_SUCCESS,
    BSP_DISPLAY_COLOR_BG);

  ctx->pan_left_btn = bsp_display_createButton(
    ctx->temp_screen, BSP_DISPLAY_TEMP_GRAPH_X + 2 * (BSP_DISPLAY_TEMP_BTN_W + BSP_DISPLAY_TEMP_BTN_GAP),
    BSP_DISPLAY_TEMP_BTN_Y, BSP_DISPLAY_TEMP_BTN_W, BSP_DISPLAY_TEMP_BTN_H, "<", BSP_DISPLAY_COLOR_PRIMARY,
    BSP_DISPLAY_COLOR_BG);

  ctx->pan_right_btn = bsp_display_createButton(
    ctx->temp_screen, BSP_DISPLAY_TEMP_GRAPH_X + 3 * (BSP_DISPLAY_TEMP_BTN_W + BSP_DISPLAY_TEMP_BTN_GAP),
    BSP_DISPLAY_TEMP_BTN_Y, BSP_DISPLAY_TEMP_BTN_W, BSP_DISPLAY_TEMP_BTN_H, ">", BSP_DISPLAY_COLOR_PRIMARY,
    BSP_DISPLAY_COLOR_BG);
}

static void bsp_display_ensureBacklightChannel(bsp_display_info_t *ctx)
{
#if defined(ESP32) && defined(TFT_BL)
  if (ctx->backlight_en)
  {
    return;
  }
  ledcAttachPin(TFT_BL, BACKLIGHT_PWM_CH);
  ledcSetup(BACKLIGHT_PWM_CH, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_BITS);
  ctx->backlight_en = true;
#else
  (void) ctx;
#endif
}

/* End of file -------------------------------------------------------- */
