/**
 * @file       sys_button.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-04-28
 * @author     Hai Tran
 *
 * @brief      System Button Layer implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_button.h"

#include "cbuffer.h"
#include "log_service.h"
#include "sys_led.h"
#include "sys_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_button, LOG_LEVEL_SYS_BUTTON)
#define SYS_BUTTON_MAX_EVENTS    10
#define SYS_BUTTON_SVC_COMMIT_MS (1000)  // gap after last press → commit cmd
#define SYS_BUTTON_SVC_ENTRY_MS  (3000)  // gap with no press → abort
#define SYS_BUTTON_SVC_MAX_MS    (30000)
#define SYS_BUTTON_SVC_CMD_MAX   (5)

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  SVC_IDLE = 0,
  SERVICE_COUNTING,
} sys_button_svc_state_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
OS_SEM_DEFINE_STATIC(sys_button);
OS_SEM_DEFINE_STATIC(sys_button_wakeup_sem);

static cbuffer_t               s_sys_button_cb;
static bsp_button_press_type_t s_sys_button_cb_buf[SYS_BUTTON_MAX_EVENTS + 1];
static sys_manager_event_t     SERVICE_CMD_MAP[SYS_BUTTON_SVC_CMD_MAX + 1];

static sys_button_svc_state_t service_state         = SVC_IDLE;
static uint8_t                service_press_accum   = 0;
static uint32_t               service_enter_ms      = 0;
static uint32_t               service_last_press_ms = 0;
static sys_led_evt_t          service_led_evt       = SYS_LED_EVT_MAX;  // raised one-shot, MAX = none
static uint32_t               service_led_raised_ms = 0;

/* Private function prototypes ---------------------------------------- */
static void sys_button_callback(bsp_button_press_type_t press_type);
static void sys_button_isr_callback(void);
static void sys_button_handle_event(bsp_button_press_type_t event);
static void sys_button_service_tick(void);
static void sys_button_led_oneshot(sys_led_evt_t evt);
static void sys_button_service_reset(void);

/* Function definitions ----------------------------------------------- */

void sys_button_init(void)
{
  OS_SEM_CREATE(sys_button);
  OS_SEM_CREATE(sys_button_wakeup_sem);

  cb_init(&s_sys_button_cb, s_sys_button_cb_buf, sizeof(s_sys_button_cb_buf));
  cb_clear(&s_sys_button_cb);

  // clang-format off
#define CMD(press_count, manager_evt) SERVICE_CMD_MAP[press_count] = manager_evt
// Count service button after enter service mode | Event triggered              |
  CMD( 1                                         , SYS_MANAGER_EVT_MAX          );
  CMD( 2                                         , SYS_MANAGER_EVT_MAX          );
  CMD( 3                                         , SYS_MANAGER_EVT_LOCKED       );
  CMD( 4                                         , SYS_MANAGER_EVT_REBOOT       );
  CMD( 5                                         , SYS_MANAGER_EVT_UNLOCKED     );
#undef CMD
  // clang-format on

  bsp_button_init(BUTTON_EVT, sys_button_callback);
  bsp_button_set_isr_callback(BUTTON_EVT, sys_button_isr_callback);
}

void sys_button_process(void)
{
  if ((g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE) && (!bsp_button_is_service_pending())
      && (service_state == SVC_IDLE))
  {
    sys_button_service_reset();  // abort service mode if the device sleeps mid-count
    OS_SEM_TAKE(sys_button_wakeup_sem, OS_MAX_DELAY);
    sys_manager_write_event(SYS_MANAGER_EVT_WAKEUP);
    LOG_DBG("Button wakeup event processed");
    return;
  }

  bsp_button_press_type_t event;

  bsp_button_process();

  if (OS_SEM_TAKE(sys_button, 20) == pdTRUE)
  {
    while (cb_read(&s_sys_button_cb, &event, sizeof(event)) == sizeof(event))
    {
      sys_button_handle_event(event);
    }
  }

  sys_button_service_tick();
}

/* Private definitions ------------------------------------------------ */
static void sys_button_isr_callback(void)
{
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE)
  {
    OS_SEM_GIVE_FROM_ISR(sys_button_wakeup_sem);
  }
}

static void sys_button_callback(bsp_button_press_type_t press_type)
{
  if (cb_space_count(&s_sys_button_cb) >= sizeof(press_type))
  {
    cb_write(&s_sys_button_cb, &press_type, sizeof(press_type));
  }

  OS_SEM_GIVE(sys_button);
}

static void sys_button_handle_event(bsp_button_press_type_t event)
{
  if (service_state == SERVICE_COUNTING)
  {
    switch (event)
    {
    case BUTTON_PRESS_SHORT:
    {
      service_press_accum += 1;
      service_last_press_ms = OS_GET_TICK();
      LOG_DBG("Service count: %d", service_press_accum);
      break;
    }
    case BUTTON_PRESS_COUNT:
    {
      service_press_accum += bsp_button_get_count(BUTTON_EVT);
      service_last_press_ms = OS_GET_TICK();
      LOG_DBG("Service count: %d", service_press_accum);
      break;
    }
    default: break;
    }
    return;
  }

  switch (event)
  {
  case BUTTON_PRESS_SHORT:
  {
    LOG_DBG("Button short press detected");
    break;
  }
  case BUTTON_PRESS_LONG:
  {
    LOG_DBG("Button long press detected");
    sys_manager_write_event(SYS_MANAGER_EVT_HELP);
    sys_button_led_oneshot(SYS_LED_EVT_HELP);
    break;
  }
  case BUTTON_PRESS_SERVICE:
  {
    LOG_DBG("Service mode entered");
    service_state         = SERVICE_COUNTING;
    service_press_accum   = 0;
    service_enter_ms      = OS_GET_TICK();
    service_last_press_ms = service_enter_ms;
    sys_button_led_oneshot(SYS_LED_EVT_SERVICE_ENTER);
    break;
  }
  case BUTTON_PRESS_COUNT:
  {
    uint8_t count = bsp_button_get_count(BUTTON_EVT);
    LOG_DBG("Button press detected: %d times", count);
    break;
  }
  default: break;
  }
}

static void sys_button_service_tick(void)
{
  uint32_t now = OS_GET_TICK();

  if (service_led_evt < SYS_LED_EVT_MAX && (now - service_led_raised_ms) >= BSP_LED_TIME_BLINK_ONCE_MS)
  {
    sys_led_clear_event(service_led_evt);
    service_led_evt = SYS_LED_EVT_MAX;
  }

  if (service_state != SERVICE_COUNTING)
  {
    return;
  }

  if ((now - service_enter_ms) >= SYS_BUTTON_SVC_MAX_MS)
  {
    LOG_DBG("Service mode timed out (max)");
    sys_button_led_oneshot(SYS_LED_EVT_SERVICE_ABORT);
    service_state = SVC_IDLE;
    return;
  }

  uint32_t gap_ms = (service_press_accum > 0) ? SYS_BUTTON_SVC_COMMIT_MS : SYS_BUTTON_SVC_ENTRY_MS;
  uint32_t ref    = (service_press_accum > 0) ? service_last_press_ms : service_enter_ms;
  if ((now - ref) < gap_ms)
  {
    return;  // still within the counting window
  }

  if (service_press_accum == 0)
  {
    LOG_DBG("Service mode aborted (no press)");
    service_state = SVC_IDLE;
    return;
  }

  if (service_press_accum <= SYS_BUTTON_SVC_CMD_MAX && SERVICE_CMD_MAP[service_press_accum] != SYS_MANAGER_EVT_MAX)
  {
    LOG_DBG("Service cmd: %d presses -> event %d", service_press_accum, SERVICE_CMD_MAP[service_press_accum]);
    sys_manager_write_event(SERVICE_CMD_MAP[service_press_accum]);
    sys_button_led_oneshot(SYS_LED_EVT_SERVICE_DONE);
  }
  else
  {
    LOG_DBG("Service cmd: %d presses unmapped / out of range", service_press_accum);
    sys_button_led_oneshot(SYS_LED_EVT_SERVICE_ABORT);
  }

  service_state = SVC_IDLE;
}

static void sys_button_led_oneshot(sys_led_evt_t evt)
{
  if (service_led_evt < SYS_LED_EVT_MAX)
  {
    sys_led_clear_event(service_led_evt);
  }
  sys_led_write_event(evt);
  service_led_evt       = evt;
  service_led_raised_ms = OS_GET_TICK();
}

static void sys_button_service_reset(void)
{
  if (service_led_evt < SYS_LED_EVT_MAX)
  {
    sys_led_clear_event(service_led_evt);
    service_led_evt = SYS_LED_EVT_MAX;
  }
  service_state       = SVC_IDLE;
  service_press_accum = 0;
}

/* End of file -------------------------------------------------------- */