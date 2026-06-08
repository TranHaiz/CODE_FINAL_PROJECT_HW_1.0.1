/**
 * @file       sys_led.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System service LED control interface
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_led.h"

#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_led, LOG_LEVEL_SYS_LED);

#define LED_RGB_TASK_MS (100)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  sys_led_evt_t curr_event;
  sys_led_evt_t prev_event;
  bool          is_active[SYS_LED_EVT_MAX];
} sys_led_handler_t;

typedef struct
{
  bsp_led_color_t color;
  bsp_led_mode_t  mode;
  uint8_t         brightness;
} sys_led_evt_info_t;

/* Private macros ----------------------------------------------------- */
#define SET_LED_EVENT(event)   sys_led_handler.is_active[event] = true
#define CLEAR_LED_EVENT(event) sys_led_handler.is_active[event] = false

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sys_led_evt_info_t SYS_LED_EVT_INFO[SYS_LED_EVT_MAX];
OS_MUTEX_DEFINE_STATIC(sys_led_event_mutex);

static sys_led_handler_t sys_led_handler;

/* Private function prototypes ---------------------------------------- */
static void sys_led_get_current_event(sys_led_evt_t *event);

/* Function definitions ----------------------------------------------- */
void sys_led_init(void)
{
  bsp_led_init(100);
  bsp_led_off();
  OS_MUTEX_CREATE(sys_led_event_mutex);
  memset(&sys_led_handler, 0, sizeof(sys_led_handler));

  // clang-format off
#define INFO(event, color, mode, brightness) SYS_LED_EVT_INFO[event] = { color, mode, brightness }
  /*Event                                 |Color                |Mode                      |Brightness   */
  INFO(SYS_LED_EVT_OFF,                    BSP_LED_COLOR_NONE,   BSP_LED_MODE_NONE,         0             );
  INFO(SYS_LED_EVT_HELP,                   BSP_LED_COLOR_YELLOW,   BSP_LED_MODE_BLINK_ONCE,   100           );
  INFO(SYS_LED_EVT_SERVICE_ENTER,          BSP_LED_COLOR_CYAN,   BSP_LED_MODE_BLINK_ONCE,   100           );
  INFO(SYS_LED_EVT_SERVICE_DONE,           BSP_LED_COLOR_GREEN,  BSP_LED_MODE_BLINK_ONCE,   100           );
  INFO(SYS_LED_EVT_SERVICE_ABORT,          BSP_LED_COLOR_RED,    BSP_LED_MODE_BLINK_ONCE,   100           );
  INFO(SYS_LED_EVT_NOTI_DANGER,            BSP_LED_COLOR_RED,    BSP_LED_MODE_FLASH_FAST,   100           );
  INFO(SYS_LED_EVT_NOTI_LOW_BALANCE,       BSP_LED_COLOR_YELLOW, BSP_LED_MODE_FLASH_SLOW,   50            );
  INFO(SYS_LED_EVT_NOTI_WARNING_DEBT,      BSP_LED_COLOR_YELLOW, BSP_LED_MODE_PULSE,        50            );
  INFO(SYS_LED_EVT_NOTI_RENTAL_LIMIT,      BSP_LED_COLOR_ORANGE, BSP_LED_MODE_FLASH_FAST,   50            );
  INFO(SYS_LED_EVT_ERROR_GPS,              BSP_LED_COLOR_ORANGE, BSP_LED_MODE_PULSE,        100           );
  INFO(SYS_LED_EVT_ERROR_FUEL_GAUGE,       BSP_LED_COLOR_PINK,   BSP_LED_MODE_FLASH_FAST,   100           );
  INFO(SYS_LED_EVT_ERROR_TEMP_HUM,         BSP_LED_COLOR_PINK,   BSP_LED_MODE_PULSE,        100           );
  INFO(SYS_LED_EVT_ERROR_IMU,              BSP_LED_COLOR_CYAN,   BSP_LED_MODE_FLASH_FAST,   100           );
  INFO(SYS_LED_EVT_ERROR_COMPASS,          BSP_LED_COLOR_CYAN,   BSP_LED_MODE_PULSE,        100           );
  INFO(SYS_LED_EVT_ERROR_SIM,              BSP_LED_COLOR_GREEN,  BSP_LED_MODE_FLASH_FAST,   100           );
  INFO(SYS_LED_EVT_ERROR_RTC,              BSP_LED_COLOR_GREEN,  BSP_LED_MODE_PULSE,        100           );
  INFO(SYS_LED_EVT_ERROR_NETWORK_LOST,     BSP_LED_COLOR_BLUE,   BSP_LED_MODE_PULSE,        100           );
#undef INFO
  // clang-format on
}

void sys_led_write_event(sys_led_evt_t event)
{
  if ((event >= SYS_LED_EVT_MAX) || (sys_led_handler.is_active[event]))
    return;

  OS_MUTEX_LOCK(sys_led_event_mutex);
  SET_LED_EVENT(event);
  OS_MUTEX_UNLOCK(sys_led_event_mutex);
}

void sys_led_clear_event(sys_led_evt_t event)
{
  if ((event >= SYS_LED_EVT_MAX) || (!sys_led_handler.is_active[event]))
    return;

  OS_MUTEX_LOCK(sys_led_event_mutex);
  CLEAR_LED_EVENT(event);
  OS_MUTEX_UNLOCK(sys_led_event_mutex);
}

void sys_led_process(void)
{
  sys_led_get_current_event(&sys_led_handler.curr_event);
  if (sys_led_handler.prev_event != sys_led_handler.curr_event)
  {
    if (sys_led_handler.curr_event < SYS_LED_EVT_MAX)
    {
      bsp_led_set(SYS_LED_EVT_INFO[sys_led_handler.curr_event].color, SYS_LED_EVT_INFO[sys_led_handler.curr_event].mode,
                  SYS_LED_EVT_INFO[sys_led_handler.curr_event].brightness);
    }
    LOG_DBG("LED event changed from %d to %d", sys_led_handler.prev_event, sys_led_handler.curr_event);
    sys_led_handler.prev_event = sys_led_handler.curr_event;
  }
  else
  {
    if (!sys_led_handler.is_active[sys_led_handler.curr_event])
    {
      bsp_led_off();
    }
  }
  bsp_led_task();
}

bool sys_led_is_event_active(sys_led_evt_t event)
{
  if (event >= SYS_LED_EVT_MAX)
    return false;

  OS_MUTEX_LOCK(sys_led_event_mutex);
  bool is_active = sys_led_handler.is_active[event];
  OS_MUTEX_UNLOCK(sys_led_event_mutex);
  return is_active;
}

/* Private definitions ----------------------------------------------- */
static void sys_led_get_current_event(sys_led_evt_t *event)
{
  OS_MUTEX_LOCK(sys_led_event_mutex);
  for (size_t i = 0; i < SYS_LED_EVT_MAX; i++)
  {
    if (sys_led_handler.is_active[i])
    {
      *event = (sys_led_evt_t) i;
      if (*event == SYS_LED_EVT_OFF)
      {
        // If OFF event is active, it takes precedence and clears some low priority events
        CLEAR_LED_EVENT(SYS_LED_EVT_NOTI_DANGER);
        CLEAR_LED_EVENT(SYS_LED_EVT_NOTI_LOW_BALANCE);
        CLEAR_LED_EVENT(SYS_LED_EVT_NOTI_WARNING_DEBT);
        CLEAR_LED_EVENT(SYS_LED_EVT_NOTI_RENTAL_LIMIT);
        CLEAR_LED_EVENT(SYS_LED_EVT_OFF);
      }
      break;
    }
  }
  OS_MUTEX_UNLOCK(sys_led_event_mutex);
}

/* End of file -------------------------------------------------------- */
