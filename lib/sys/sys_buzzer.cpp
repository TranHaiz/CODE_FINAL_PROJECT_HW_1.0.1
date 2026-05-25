/**
 * @file       sys_buzzer.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-25
 * @author     Hai Tran
 *
 * @brief     System buzzer with priority-based event queue
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_buzzer.h"

#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_buzzer, LOG_LEVEL_SYS_BUZZER);

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  SYS_BUZZER_PATTERN_NONE = 0,
  SYS_BUZZER_PATTERN_SOLID,       // continuous tone via bsp_buzzer_enable(true)
  SYS_BUZZER_PATTERN_BEEP_LONG,   // single beep, duration_ms
  SYS_BUZZER_PATTERN_BEEP_CYCLE,  // N cycles, beep_ms ON within period_ms
} sys_buzzer_pattern_t;

typedef struct
{
  sys_buzzer_pattern_t pattern;
  uint32_t             beep_ms;
  uint32_t             period_ms;
  uint8_t              cycles;
} sys_buzzer_evt_info_t;

typedef struct
{
  sys_buzzer_evt_t curr_event;
  sys_buzzer_evt_t prev_event;
  bool             is_active[SYS_BUZZER_EVT_MAX];
} sys_buzzer_handler_t;

/* Private macros ----------------------------------------------------- */
#define SET_BUZZER_EVENT(event)   sys_buzzer_handler.is_active[event] = true
#define CLEAR_BUZZER_EVENT(event) sys_buzzer_handler.is_active[event] = false

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sys_buzzer_evt_info_t SYS_BUZZER_EVT_INFO[SYS_BUZZER_EVT_MAX];
OS_MUTEX_DEFINE_STATIC(sys_buzzer_event_mutex);

static sys_buzzer_handler_t sys_buzzer_handler;

/* Private function prototypes ---------------------------------------- */
static void sys_buzzer_get_current_event(sys_buzzer_evt_t *event);
static void sys_buzzer_dispatch(sys_buzzer_evt_t event);

/* Function definitions ----------------------------------------------- */
void sys_buzzer_init(void)
{
  bsp_buzzer_init();
  OS_MUTEX_CREATE(sys_buzzer_event_mutex);
  memset(&sys_buzzer_handler, 0, sizeof(sys_buzzer_handler));

  // clang-format off
#define INFO(event, pattern, beep_ms, period_ms, cycles) \
  SYS_BUZZER_EVT_INFO[event] = { pattern, beep_ms, period_ms, cycles }
  /*Event                          |Pattern                         |beep_ms|period_ms|cycles */
  INFO(SYS_BUZZER_EVT_OFF,          SYS_BUZZER_PATTERN_NONE,          0,      0,        0      );
  INFO(SYS_BUZZER_EVT_STOLEN,       SYS_BUZZER_PATTERN_SOLID,         0,      0,        0      );
  INFO(SYS_BUZZER_EVT_RENTAL_LIMIT, SYS_BUZZER_PATTERN_BEEP_CYCLE,    1000,   2000,     UINT8_MAX);
  INFO(SYS_BUZZER_EVT_LOW_BATT,     SYS_BUZZER_PATTERN_BEEP_LONG,     500,    0,        0      );
  INFO(SYS_BUZZER_EVT_WARN_DEBT,    SYS_BUZZER_PATTERN_BEEP_CYCLE,    500,    1500,     1      );
  INFO(SYS_BUZZER_EVT_LOW_BALANCE,  SYS_BUZZER_PATTERN_BEEP_LONG,     500,    0,        0      );
  INFO(SYS_BUZZER_EVT_STARTUP,      SYS_BUZZER_PATTERN_BEEP_CYCLE,    200,    500,      3      );
#undef INFO
  // clang-format on
}

void sys_buzzer_write_event(sys_buzzer_evt_t event)
{
  if ((event >= SYS_BUZZER_EVT_MAX) || (sys_buzzer_handler.is_active[event]))
    return;

  OS_MUTEX_LOCK(sys_buzzer_event_mutex);
  SET_BUZZER_EVENT(event);
  OS_MUTEX_UNLOCK(sys_buzzer_event_mutex);
}

void sys_buzzer_clear_event(sys_buzzer_evt_t event)
{
  if ((event >= SYS_BUZZER_EVT_MAX) || (!sys_buzzer_handler.is_active[event]))
    return;

  OS_MUTEX_LOCK(sys_buzzer_event_mutex);
  CLEAR_BUZZER_EVENT(event);
  OS_MUTEX_UNLOCK(sys_buzzer_event_mutex);
}

void sys_buzzer_process(void)
{
  sys_buzzer_get_current_event(&sys_buzzer_handler.curr_event);
  if (sys_buzzer_handler.prev_event != sys_buzzer_handler.curr_event)
  {
    sys_buzzer_dispatch(sys_buzzer_handler.curr_event);
    LOG_DBG("Buzzer event changed from %d to %d", sys_buzzer_handler.prev_event, sys_buzzer_handler.curr_event);
    sys_buzzer_handler.prev_event = sys_buzzer_handler.curr_event;
  }
  bsp_buzzer_process();
}

/* Private definitions ----------------------------------------------- */
static void sys_buzzer_get_current_event(sys_buzzer_evt_t *event)
{
  OS_MUTEX_LOCK(sys_buzzer_event_mutex);
  *event = SYS_BUZZER_EVT_OFF;  // default if nothing active
  for (size_t i = 0; i < SYS_BUZZER_EVT_MAX; i++)
  {
    if (sys_buzzer_handler.is_active[i])
    {
      *event = (sys_buzzer_evt_t) i;
      if (*event == SYS_BUZZER_EVT_OFF)
      {
        // OFF takes precedence and silences low-priority noti events
        CLEAR_BUZZER_EVENT(SYS_BUZZER_EVT_RENTAL_LIMIT);
        CLEAR_BUZZER_EVENT(SYS_BUZZER_EVT_LOW_BATT);
        CLEAR_BUZZER_EVENT(SYS_BUZZER_EVT_WARN_DEBT);
        CLEAR_BUZZER_EVENT(SYS_BUZZER_EVT_LOW_BALANCE);
        CLEAR_BUZZER_EVENT(SYS_BUZZER_EVT_OFF);
      }
      break;
    }
  }
  OS_MUTEX_UNLOCK(sys_buzzer_event_mutex);
}

static void sys_buzzer_dispatch(sys_buzzer_evt_t event)
{
  if (event >= SYS_BUZZER_EVT_MAX)
  {
    bsp_buzzer_enable(false);
    return;
  }
  const sys_buzzer_evt_info_t *info = &SYS_BUZZER_EVT_INFO[event];
  switch (info->pattern)
  {
  case SYS_BUZZER_PATTERN_SOLID: bsp_buzzer_enable(true); break;
  case SYS_BUZZER_PATTERN_BEEP_LONG: bsp_buzzer_beep_long(info->beep_ms); break;
  case SYS_BUZZER_PATTERN_BEEP_CYCLE: bsp_buzzer_beep_cycle(info->cycles, info->beep_ms, info->period_ms); break;
  case SYS_BUZZER_PATTERN_NONE:
  default: bsp_buzzer_enable(false); break;
  }
}

/* End of file -------------------------------------------------------- */
