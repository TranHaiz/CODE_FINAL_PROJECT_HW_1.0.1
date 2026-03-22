/**
 * @file       bsp_led.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     BSP implementation for WWS120 RGB LED
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_led.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"

#include <Adafruit_NeoPixel.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_led, LOG_LEVEL_DBG);

#define LED_MAX_VALUE                 (255)
#define LED_OFF_VALUE                 (0)
#define BSP_LED_PULSE_FADE_TICKS_HIGH (BSP_LED_TIME_PULSE_HIGH_SOLID_MS / led_tick_ms)
#define BSP_LED_PULSE_FADE_TICKS_LOW  (BSP_LED_TIME_PULSE_LOW_SOLID_MS / led_tick_ms)
#define BSP_LED_PULSE_HOLD_TICKS_HIGH (BSP_LED_TIME_PULSE_HIGH_MS / led_tick_ms)
#define BSP_LED_PULSE_HOLD_TICKS_LOW  (BSP_LED_TIME_PULSE_LOW_MS / led_tick_ms)
#define BSP_LED_COUNT                 (1)

/* Private macros ----------------------------------------------------- */
typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} led_color_info_t;

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static led_color_info_t         LED_COLOR_INFO[LED_COLOR_MAX];
static Adafruit_NeoPixel        led_strip(BSP_LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
static volatile bsp_led_mode_t  led_mode       = LED_MODE_SOLID;
static volatile bsp_led_color_t led_color      = LED_COLOR_GREEN;
static volatile uint8_t         led_brightness = 100;
static volatile int             pulse_val      = 0;
static volatile int             fade_amount    = 5;
static volatile uint32_t        flash_count    = 0;
static volatile bool            flash_on       = false;
static uint32_t                 led_tick_ms    = 1;

/* Private function prototypes ---------------------------------------- */
static uint32_t bsp_led_make_color(bsp_led_color_t color, uint8_t brightness);
static void     bsp_led_update_task(void);

/* Function definitions ----------------------------------------------- */
void bsp_led_init(uint32_t tick_ms)
{
  led_strip.begin();
  led_strip.setBrightness(LED_MAX_VALUE);
  led_strip.show();
  led_tick_ms = tick_ms;

  // clang-format off
#define INFO(color, r, g, b) LED_COLOR_INFO[color] = { r, g, b }
  //        LED_COLOR_NAME      R     G       B
  INFO(LED_COLOR_GREEN,         0,    255,    0);
  INFO(LED_COLOR_BLUE,          0,    0,      255);
  INFO(LED_COLOR_YELLOW,        255,  255,    0);
  INFO(LED_COLOR_CYAN,          0,    255,    255);
  INFO(LED_COLOR_WHITE,         255,  255,    255);
  INFO(LED_COLOR_RED,           255,  0,      0);
  INFO(LED_COLOR_PINK,          255,  192,    203);
  INFO(LED_COLOR_PURPLE,        128,  0,      128);
#undef INFO
  // clang-format on
}

void bsp_led_set(bsp_led_color_t color, bsp_led_mode_t mode_in, uint8_t brightness)
{
  if (color >= LED_COLOR_MAX)
  {
    LOG_ERR("Invalid LED color: %d", color);
    return;
  }
  if (brightness > 100)
  {
    LOG_ERR("Invalid LED brightness: %d", brightness);
    return;
  }

  led_color      = color;
  led_brightness = brightness;
  led_mode       = mode_in;

  // Reset animation state
  pulse_val   = 0;
  fade_amount = 5;
  flash_count = 0;
  flash_on    = false;
}

/**
 * @brief Tick function, should be called in main loop to update LED
 */
void bsp_led_task(void)
{
  bsp_led_update_task();
  if (led_strip.canShow())
  {
    led_strip.show();
  }
}

void bsp_led_off(void)
{
  led_strip.setPixelColor(0, 0);
  led_strip.show();
}

/* Private definitions ----------------------------------------------- */
static uint32_t bsp_led_make_color(bsp_led_color_t color, uint8_t brightness)
{
  uint8_t r, g, b;

  r = LED_COLOR_INFO[color].r;
  g = LED_COLOR_INFO[color].g;
  b = LED_COLOR_INFO[color].b;
  r = (uint8_t) ((r * brightness) / 100);
  g = (uint8_t) ((g * brightness) / 100);
  b = (uint8_t) ((b * brightness) / 100);
  return led_strip.Color(r, g, b);
}

static void bsp_led_update_task(void)
{
  static uint32_t last_tick_ms = 0;
  uint32_t        now_ms       = OS_GET_TICK();
  uint32_t        delta_ms     = now_ms - last_tick_ms;
  last_tick_ms                 = now_ms;

  switch (led_mode)
  {
  case LED_MODE_SOLID:
  {
    led_strip.setPixelColor(0, bsp_led_make_color(led_color, led_brightness));
    break;
  }
  case LED_MODE_FLASH_SLOW:
  {
    static uint32_t flash_tick_slow = 0;
    flash_tick_slow += delta_ms;
    if (flash_tick_slow >= BSP_LED_TIME_FLASH_SLOW_MS)
    {
      flash_tick_slow = 0;
      flash_on        = !flash_on;
      LOG_DBG("Flash slow state changed: %s", flash_on ? "ON" : "OFF");
      led_strip.setPixelColor(0, (flash_on ? bsp_led_make_color(led_color, led_brightness) : LED_OFF_VALUE));
    }
    break;
  }
  case LED_MODE_FLASH_FAST:
  {
    static uint32_t flash_tick_fast = 0;
    flash_tick_fast += delta_ms;
    if (flash_tick_fast >= BSP_LED_TIME_FLASH_FAST_MS)
    {
      flash_tick_fast = 0;
      flash_on        = !flash_on;
      led_strip.setPixelColor(0, (flash_on ? bsp_led_make_color(led_color, led_brightness) : LED_OFF_VALUE));
    }
    break;
  }
  case LED_MODE_PULSE:
  {
    static uint32_t hold_ticks = 0;

    if (fade_amount > 0)
    {
      pulse_val += (LED_MAX_VALUE / BSP_LED_PULSE_FADE_TICKS_HIGH);
      if (pulse_val >= LED_MAX_VALUE)
      {
        pulse_val   = LED_MAX_VALUE;
        hold_ticks  = 0;
        fade_amount = 0;
      }
    }
    else if (fade_amount < 0)
    {
      pulse_val -= (LED_MAX_VALUE / BSP_LED_PULSE_FADE_TICKS_LOW);
      if (pulse_val <= 0)
      {
        pulse_val   = 0;
        hold_ticks  = 0;
        fade_amount = 0;
      }
    }
    else
    {
      hold_ticks++;
      if (pulse_val == LED_MAX_VALUE && hold_ticks >= BSP_LED_PULSE_HOLD_TICKS_HIGH)
      {
        fade_amount = -1;
      }
      else if (pulse_val == 0 && hold_ticks >= BSP_LED_PULSE_HOLD_TICKS_LOW)
      {
        fade_amount = 1;
      }
    }

    uint8_t pulse_brightness = (uint8_t) ((pulse_val * led_brightness) / LED_MAX_VALUE);
    led_strip.setPixelColor(0, bsp_led_make_color(led_color, pulse_brightness));
    break;
  }
  }
}

/* End of file -------------------------------------------------------- */
