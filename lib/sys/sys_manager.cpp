/**
 * @file       sys_manager.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System manager to maintain overall system state and device information
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_manager.h"

#include "bsp_acc.h"
#include "bsp_device.h"
#include "bsp_led.h"
#include "bsp_sdcard.h"
#include "bsp_servo.h"
#include "bsp_sim.h"
#include "bsp_timer.h"
#include "cbuffer.h"
#include "device_info.h"
#include "sys_buzzer.h"
#include "sys_input.h"
#include "sys_led.h"
#include "sys_log.h"
#include "sys_network.h"
#include "sys_network_adapter_lte.h"
#include "sys_ui.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_manager, LOG_LEVEL_SYS_MANAGER)

#if (DEVICE_IDLE_MODE_ENABLED)
#define SHUTDOWN_TIMER_PERIOD_MS (30000)
#endif  // DEVICE_IDLE_MODE_ENABLED

#define DEVICE_DANGER_NOTI_INTERVAL_MS     (15000)
#define DEVICE_LOW_BALANCE_NOTI_TIMEOUT_MS (5000)
#define DEVICE_STOLEN_TIMEOUT_MS           (30000)  // no motion in STOLEN => auto exit to LOCKED
#define DEVICE_PAUSE_CONFIRM_TIMEOUT_MS    (20000)  // wait for server "OK" after pause, else abort
#define USER_ACTION_COOLDOWN_MS            (3000)   // anti-spam for stop/pause buttons

/* Private enumerate/structure ---------------------------------------- */
typedef void (*sys_manager_process_handler_t)(void);

#define SYS_MANAGER_EVENT_QUEUE_SIZE 20

typedef struct
{
  cbuffer_t                     event_cb;
  sys_manager_process_handler_t handler[SYS_MANAGER_EVT_MAX];
  bsp_timer_t                   shutdown_timer;
  bsp_timer_t                   danger_noti_timer;
  bsp_timer_t                   low_balance_noti_timer;
  bsp_timer_t                   stolen_timeout_timer;
  bsp_timer_t                   pause_timeout_timer;
  bool                          is_noti_limited_active;
  bool                          is_warning_debt_active;
  bool                          is_danger_noti_active;
  bool                          is_pause_pending;
  uint32_t                      last_user_action_ms;
} sys_manager_handler_t;

/* Private macros ----------------------------------------------------- */
#define INFO(event, func) manager_handler.handler[event] = func

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
OS_SEM_DEFINE_STATIC(sys_manager_event_sem);
OS_MUTEX_DEFINE_STATIC(sys_manager_event_mutex);
static sys_manager_handler_t manager_handler;
static sys_manager_event_t   s_event_buffer[SYS_MANAGER_EVENT_QUEUE_SIZE];
static bool                  s_manager_ready = false;  // event queue (sem/mutex) created

/* Private function prototypes ---------------------------------------- */
static void sys_manager_wakeup_handler(void);
static void sys_manager_lock_handler(void);
static void sys_manager_active_handler(void);
static void sys_manager_unlocked_handler(void);
static void sys_manager_change_topic_sub_handler(void);
static void sys_manager_reboot_handler(void);
static void sys_manager_user_lock_handler(void);
static void sys_manager_user_pause_handler(void);
static void sys_manager_pause_confirm_handler(void);
static void sys_manager_pause_timeout_handler(void);
static void sys_manager_pause_timeout_timer_callback(TimerHandle_t xTimer);
static void sys_manager_shutdown_timer_callback(TimerHandle_t xTimer);
static void sys_manager_shutdown_handler(void);
static void sys_manager_device_stolen_handler(void);
static void sys_manager_unlock_from_network_handler(void);
static void sys_manager_lock_from_network_handler(void);
static void sys_manager_start_rental_handler(void);
static void sys_manager_stop_rental_fail_handler(void);
static void sys_manager_stop_rental_success_handler(void);
static void sys_manager_reset_offline_data_handler(void);
static void sys_manager_danger_noti_timer_callback(TimerHandle_t xTimer);
static void sys_manager_stop_stolen_noti(void);
static void sys_manager_stolen_timeout_timer_callback(TimerHandle_t xTimer);
static void sys_manager_stolen_timeout_handler(void);
static void sys_manager_flush_log(void);
static void sys_manager_rental_noti_limit_handler(void);
static void sys_manager_warn_debt_handler(void);
static void sys_manager_clear_debt_handler(void);
static void sys_manager_warn_low_balance_handler(void);
static void sys_manager_noti_low_batt_handler(void);
static void sys_manager_low_balance_noti_timer_callback(TimerHandle_t xTimer);
static void sys_manager_help_handler(void);
static void sys_manager_check_lte_band_handler(void);
static void sys_manager_apply_danger_noti_handler(void);
static void sys_manager_sync_lock_handler(void);

/* Function definitions ----------------------------------------------- */
void sys_manager_init(void)
{
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_init(&manager_handler.shutdown_timer, SHUTDOWN_TIMER_PERIOD_MS, false, sys_manager_shutdown_timer_callback);
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_LOCKED
      || g_device_info.nvs_info.curr_state == DEVICE_STATE_PAUSED)
    bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED

  bsp_timer_init(&manager_handler.danger_noti_timer, DEVICE_DANGER_NOTI_INTERVAL_MS, false,
                 sys_manager_danger_noti_timer_callback);

  bsp_timer_init(&manager_handler.low_balance_noti_timer, DEVICE_LOW_BALANCE_NOTI_TIMEOUT_MS, false,
                 sys_manager_low_balance_noti_timer_callback);

  bsp_timer_init(&manager_handler.stolen_timeout_timer, DEVICE_STOLEN_TIMEOUT_MS, false,
                 sys_manager_stolen_timeout_timer_callback);

  bsp_timer_init(&manager_handler.pause_timeout_timer, DEVICE_PAUSE_CONFIRM_TIMEOUT_MS, false,
                 sys_manager_pause_timeout_timer_callback);

  OS_SEM_CREATE(sys_manager_event_sem);
  OS_MUTEX_CREATE(sys_manager_event_mutex);
  cb_init(&manager_handler.event_cb, s_event_buffer, sizeof(s_event_buffer));

  // clang-format off
  /*   Event                                |   Handlers*/
  INFO(SYS_MANAGER_EVT_WAKEUP               ,   sys_manager_wakeup_handler              );
  INFO(SYS_MANAGER_EVT_LOCKED               ,   sys_manager_lock_handler                );
  INFO(SYS_MANAGER_EVT_UNLOCKED             ,   sys_manager_unlocked_handler            );
  INFO(SYS_MANAGER_EVT_ACTIVE               ,   sys_manager_active_handler              );
  INFO(SYS_MANAGER_EVT_CHANGE_CMD_TOPIC     ,   sys_manager_change_topic_sub_handler    );
  INFO(SYS_MANAGER_EVT_REBOOT               ,   sys_manager_reboot_handler              );
  INFO(SYS_MANAGER_EVT_USER_LOCK            ,   sys_manager_user_lock_handler           );
  INFO(SYS_MANAGER_EVT_USER_PAUSE           ,   sys_manager_user_pause_handler          );
  INFO(SYS_MANAGER_EVT_PAUSE_CONFIRM        ,   sys_manager_pause_confirm_handler       );
  INFO(SYS_MANAGER_EVT_PAUSE_TIMEOUT        ,   sys_manager_pause_timeout_handler       );
  INFO(SYS_MANAGER_EVT_SHUTDOWN             ,   sys_manager_shutdown_handler            );
  INFO(SYS_MANAGER_EVT_DEVICE_STOLEN        ,   sys_manager_device_stolen_handler       );
  INFO(SYS_MANAGER_EVT_UNLOCK_FROM_NETWORK  ,   sys_manager_unlock_from_network_handler );
  INFO(SYS_MANAGER_EVT_LOCK_FROM_NETWORK    ,   sys_manager_lock_from_network_handler   );
  INFO(SYS_MANAGER_EVT_START_RENTAL         ,   sys_manager_start_rental_handler        );
  INFO(SYS_MANAGER_EVT_STOP_RENTAL_FAIL     ,   sys_manager_stop_rental_fail_handler    );
  INFO(SYS_MANAGER_EVT_STOP_RENTAL_SUCCESS  ,   sys_manager_stop_rental_success_handler );
  INFO(SYS_MANAGER_EVT_RESET_OFFLINE_DATA   ,   sys_manager_reset_offline_data_handler  );
  INFO(SYS_MANAGER_EVT_STOP_STOLEN_NOTI     ,   sys_manager_stop_stolen_noti            );
  INFO(SYS_MANAGER_EVT_STOLEN_TIMEOUT       ,   sys_manager_stolen_timeout_handler      );
  INFO(SYS_MANAGER_EVT_FLUSH_LOG            ,   sys_manager_flush_log                   );
  INFO(SYS_MANAGER_RENTAL_NOTI_LIMIT        ,   sys_manager_rental_noti_limit_handler   );
  INFO(SYS_MANAGER_EVT_WARN_LOW_BALANCE     ,   sys_manager_warn_low_balance_handler    );
  INFO(SYS_MANAGER_EVT_NOTI_LOW_BATT        ,   sys_manager_noti_low_batt_handler       );
  INFO(SYS_MANAGER_EVT_WARN_DEBT            ,   sys_manager_warn_debt_handler           );
  INFO(SYS_MANAGER_EVT_CLEAR_DEBT           ,   sys_manager_clear_debt_handler          );
  INFO(SYS_MANAGER_EVT_HELP                 ,   sys_manager_help_handler                );
  INFO(SYS_MANAGER_EVT_CHECK_LTE_BAND       ,   sys_manager_check_lte_band_handler      );
  INFO(SYS_MANAGER_EVT_APPLY_DANGER_NOTI    ,   sys_manager_apply_danger_noti_handler   );
  INFO(SYS_MANAGER_EVT_SYNC_LOCK            ,   sys_manager_sync_lock_handler           );
  // clang-format on

  s_manager_ready = true;  // safe to accept events only after the queue and handlers exist
}
#undef INFO

void sys_manager_write_event(sys_manager_event_t event)
{
  if (!s_manager_ready)
  {
    return;
  }
  if (event < SYS_MANAGER_EVT_MAX)
  {
    OS_MUTEX_LOCK(sys_manager_event_mutex);
    size_t written = cb_write(&manager_handler.event_cb, &event, sizeof(sys_manager_event_t));
    if (written == sizeof(sys_manager_event_t))
    {
      OS_MUTEX_UNLOCK(sys_manager_event_mutex);
      OS_SEM_GIVE(sys_manager_event_sem);
    }
    else
    {
      OS_MUTEX_UNLOCK(sys_manager_event_mutex);
      LOG_WRN("sys_manager event_cb FULL!");
    }
  }
}

void sys_manager_process(void)
{
  OS_SEM_TAKE(sys_manager_event_sem, OS_MAX_DELAY);

  sys_manager_event_t event_to_process = SYS_MANAGER_EVT_MAX;

  // Drain the whole queue: the event semaphore is binary, so rapid gives coalesce
  // and a single take must not leave queued events stuck until the next give.
  while (true)
  {
    OS_MUTEX_LOCK(sys_manager_event_mutex);
    size_t read = cb_read(&manager_handler.event_cb, &event_to_process, sizeof(sys_manager_event_t));
    OS_MUTEX_UNLOCK(sys_manager_event_mutex);

    if (read != sizeof(sys_manager_event_t))
    {
      break;
    }

    if (event_to_process < SYS_MANAGER_EVT_MAX && manager_handler.handler[event_to_process] != nullptr)
    {
      LOG_DBG("Processed event: %d", event_to_process);
      manager_handler.handler[event_to_process]();
    }
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_manager_wakeup_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_IDLE)
    return;

  LOG_DBG("Handling wakeup event");
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_wakeup();
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
}

static void sys_manager_lock_handler(void)
{
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_lock();
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
  LOG_DBG("Device locked");
}

static void sys_manager_active_handler(void)
{
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_reset(&manager_handler.shutdown_timer);
  bsp_timer_stop(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED

  device_info_update_state(DEVICE_STATE_ACTIVE);
  sys_ui_wakeup();
  sys_input_wakeup();
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_stolen_noti();
  LOG_DBG("Handling active event");
}

static void sys_manager_unlocked_handler(void)
{
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_reset(&manager_handler.shutdown_timer);
  bsp_timer_stop(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED

  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
  {
    device_info_update_state(DEVICE_STATE_ACTIVE);
    sys_input_clear_data_for_new_rental();
    sys_ui_wakeup();
    sys_ui_unlock();
    sys_input_wakeup();
    LOG_DBG("Device unlocked and active");
  }
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_stolen_noti();
  sys_ui_noti_clear_all();
}

static void sys_manager_change_topic_sub_handler(void)
{
  LOG_DBG("Changing MQTT sub cmd change %s to new topic: %s", g_device_info.last_mqtt_cmd_topic,
          g_device_info.mqtt_cmd_topic);
  bsp_sim_mqtt_unsub(g_device_info.last_mqtt_cmd_topic);
  bsp_sim_mqtt_sub(g_device_info.mqtt_cmd_topic, sys_network_adapter_lte_mqtt_cb);
  strncpy(g_device_info.last_mqtt_cmd_topic, g_device_info.mqtt_cmd_topic,
          sizeof(g_device_info.last_mqtt_cmd_topic) - 1);
  LOG_DBG("MQTT last cmd topic: %s", g_device_info.last_mqtt_cmd_topic);
}

static void sys_manager_reboot_handler(void)
{
  bsp_device_info_save(&g_device_info.nvs_info);
  LOG_INF("---------- Rebooting device ----------");
  bsp_device_reboot();
}

static void sys_manager_user_lock_handler(void)
{
  if (!sys_network_is_trip_active())
  {
    LOG_DBG("User lock ignored: no active trip");
    return;
  }
  uint32_t now = OS_GET_TICK();
  if ((now - manager_handler.last_user_action_ms) < USER_ACTION_COOLDOWN_MS)
  {
    LOG_DBG("User lock ignored: cooldown");
    return;
  }
  manager_handler.last_user_action_ms = now;

  LOG_DBG("Handling user lock event");
  sys_network_publish_noti(NETWORK_NOTI_USERLOCK_PAYLOAD, strlen(NETWORK_NOTI_USERLOCK_PAYLOAD));
#if (DEVICE_LOCK_DEBUG_MODE_ENABLED)
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_lock();

#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
#endif  // DEVICE_LOCK_DEBUG_MODE_ENABLED
}

static void sys_manager_user_pause_handler(void)
{
  if (!sys_network_is_trip_active())
  {
    LOG_DBG("User pause ignored: no active trip");
    return;
  }
  if (manager_handler.is_pause_pending)
  {
    return;  // already waiting for server OK, ignore repeated presses
  }
  uint32_t now = OS_GET_TICK();
  if ((now - manager_handler.last_user_action_ms) < USER_ACTION_COOLDOWN_MS)
  {
    LOG_DBG("User pause ignored: cooldown");
    return;
  }
  manager_handler.last_user_action_ms = now;

  LOG_DBG("Handling user pause event");
  sys_network_publish_noti(NETWORK_NOTI_USERPAUSE_PAYLOAD, strlen(NETWORK_NOTI_USERPAUSE_PAYLOAD));
#if (DEVICE_LOCK_DEBUG_MODE_ENABLED)
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_lock();

#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
#else
  // Lock the screen only once the server confirms "OK" within the timeout window
  manager_handler.is_pause_pending = true;
  bsp_timer_start(&manager_handler.pause_timeout_timer);
#endif  // DEVICE_LOCK_DEBUG_MODE_ENABLED
}

static void sys_manager_pause_timeout_timer_callback(TimerHandle_t xTimer)
{
  sys_manager_write_event(SYS_MANAGER_EVT_PAUSE_TIMEOUT);
}

static void sys_manager_pause_confirm_handler(void)
{
  if (!manager_handler.is_pause_pending)
  {
    return;  // stray OK outside the pause window
  }
  manager_handler.is_pause_pending = false;
  bsp_timer_stop(&manager_handler.pause_timeout_timer);

  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_lock();
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
  LOG_DBG("Pause confirmed by server, screen locked");
}

static void sys_manager_pause_timeout_handler(void)
{
  if (!manager_handler.is_pause_pending)
  {
    return;
  }
  manager_handler.is_pause_pending = false;
  LOG_DBG("Pause confirm timeout, no server OK");
}

static void sys_manager_shutdown_timer_callback(TimerHandle_t xTimer)
{
  sys_manager_write_event(SYS_MANAGER_EVT_SHUTDOWN);
}

static void sys_manager_shutdown_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_IDLE)
  {
    bsp_device_info_save(&g_device_info.nvs_info);
    device_info_update_state(DEVICE_STATE_IDLE);
    bsp_acc_enable_interrupt(BSP_ACC_INT_PIN_1);
    LOG_INF("---------- Device go to idle mode ----------");
  }
}

static void sys_manager_device_stolen_handler(void)
{
  if (!g_device_info.danger_noti_enabled)
    return;

  g_device_info.danger_level = DEVICE_DANGER_LEVEL_HIGH;
  bsp_acc_enable_interrupt(BSP_ACC_INT_PIN_1);

  switch (g_device_info.nvs_info.curr_state)
  {
  case DEVICE_STATE_IDLE:
  case DEVICE_STATE_LOCKED:
  case DEVICE_STATE_PAUSED:
  {
    device_info_update_state(DEVICE_STATE_STOLEN);
    sys_buzzer_write_event(SYS_BUZZER_EVT_STOLEN);
    sys_led_write_event(SYS_LED_EVT_NOTI_DANGER);
    sys_input_wakeup();
    sys_network_wakeup();
    bsp_timer_start(&manager_handler.stolen_timeout_timer);
    break;
  }
  case DEVICE_STATE_STOLEN:
  {
    bsp_timer_reset(&manager_handler.stolen_timeout_timer);  // motion still happening => extend window
    break;
  }
  case DEVICE_STATE_ACTIVE:
  {
    // TODO: Noti to server
    break;
  }
  default: break;
  }
  bsp_timer_start(&manager_handler.danger_noti_timer);
  if (!manager_handler.is_danger_noti_active)
  {
    manager_handler.is_danger_noti_active = true;
    sys_network_publish_noti(NETWORK_NOTI_DEVICE_STOLEN, strlen(NETWORK_NOTI_DEVICE_STOLEN));
  }
}

void sys_manager_unlock_from_network_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
  {
#if (DEVICE_IDLE_MODE_ENABLED)
    bsp_timer_reset(&manager_handler.shutdown_timer);
    bsp_timer_stop(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
    // sys_network_trigger_new_trip();
    device_info_update_state(DEVICE_STATE_ACTIVE);
    if (g_device_info.nvs_info.prev_state == DEVICE_STATE_IDLE
        || g_device_info.nvs_info.prev_state == DEVICE_STATE_STOLEN)
    {
      sys_ui_wakeup();
      sys_input_wakeup();
    }
    sys_ui_unlock();
    LOG_DBG("Device unlocked and active from network");
  }
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_stolen_noti();
  sys_ui_noti_clear_all();
  sys_input_clear_data_for_new_rental();
  sys_network_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
}

void sys_manager_lock_from_network_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_LOCKED)
  {
    bool was_stolen = (g_device_info.nvs_info.curr_state == DEVICE_STATE_STOLEN);
    device_info_update_state(DEVICE_STATE_LOCKED);
    sys_ui_lock();
#if (DEVICE_IDLE_MODE_ENABLED)
    bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
    if (was_stolen)
    {
      bsp_timer_stop(&manager_handler.stolen_timeout_timer);
      bsp_timer_stop(&manager_handler.danger_noti_timer);
      manager_handler.is_danger_noti_active = false;
      g_device_info.danger_level            = DEVICE_DANGER_LEVEL_LOW;
      sys_led_write_event(SYS_LED_EVT_OFF);
      sys_buzzer_clear_event(SYS_BUZZER_EVT_STOLEN);
    }
    LOG_DBG("Device locked from network");
  }
  sys_network_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
}

static void sys_manager_start_rental_handler(void)
{
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_STOLEN)
  {
    LOG_WRN("Start rental rejected: device is STOLEN");
    return;
  }
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
  {
#if (DEVICE_IDLE_MODE_ENABLED)
    bsp_timer_reset(&manager_handler.shutdown_timer);
    bsp_timer_stop(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
    sys_network_trigger_new_trip();
    device_info_update_state(DEVICE_STATE_ACTIVE);
    if (g_device_info.nvs_info.prev_state == DEVICE_STATE_IDLE)
    {
      sys_ui_wakeup();
      sys_input_wakeup();
    }
    g_device_info.danger_noti_enabled = true;
    g_device_info.danger_level        = DEVICE_DANGER_LEVEL_LOW;
    sys_input_reset_data_for_new_rental();
    sys_ui_unlock();
    LOG_DBG("Device unlocked and active from network");
  }
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_stolen_noti();
  sys_ui_noti_clear_all();
  sys_input_clear_data_for_new_rental();
  sys_network_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
}

static void sys_manager_stop_rental_fail_handler(void)
{
  device_info_update_state(DEVICE_STATE_ACTIVE);
  sys_ui_noti_set(SYS_UI_NOTI_LABEL_OUT_OF_ZONE);
}

static void sys_manager_stop_rental_success_handler(void)
{
  sys_ui_noti_clear_all();
  sys_network_trigger_end_trip();
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_network_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
  if (manager_handler.is_noti_limited_active || manager_handler.is_warning_debt_active)
  {
    manager_handler.is_warning_debt_active = false;
    manager_handler.is_noti_limited_active = false;
    sys_led_write_event(SYS_LED_EVT_OFF);
    sys_buzzer_write_event(SYS_BUZZER_EVT_OFF);
  }

  // Reset danger notification settings to default values when rental stops
  g_device_info.danger_noti_enabled = true;
  g_device_info.danger_level        = DEVICE_DANGER_LEVEL_LOW;
}

static void sys_manager_reset_offline_data_handler(void)
{
  sys_network_reset_offline_data();
  OS_DELAY_MS(1000);
  bsp_device_reboot();
}

static void sys_manager_danger_noti_timer_callback(TimerHandle_t xTimer)
{
  sys_manager_write_event(SYS_MANAGER_EVT_STOP_STOLEN_NOTI);
}

static void sys_manager_stop_stolen_noti(void)
{
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_STOLEN)
  {
    bsp_timer_start(&manager_handler.danger_noti_timer);  // keep buzzer+LED until stolen clears
    return;
  }
  bsp_timer_stop(&manager_handler.stolen_timeout_timer);
  manager_handler.is_danger_noti_active = false;
  sys_led_write_event(SYS_LED_EVT_OFF);
  sys_ui_wakeup();
  sys_buzzer_clear_event(SYS_BUZZER_EVT_STOLEN);
}

static void sys_manager_stolen_timeout_timer_callback(TimerHandle_t xTimer)
{
  sys_manager_write_event(SYS_MANAGER_EVT_STOLEN_TIMEOUT);
}

static void sys_manager_stolen_timeout_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_STOLEN)
    return;

  LOG_DBG("Stolen timeout: no motion => back to LOCKED");
  bsp_timer_stop(&manager_handler.danger_noti_timer);
  manager_handler.is_danger_noti_active = false;
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_led_write_event(SYS_LED_EVT_OFF);
  sys_buzzer_clear_event(SYS_BUZZER_EVT_STOLEN);
  sys_ui_lock();
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif
}

static void sys_manager_flush_log(void)
{
  sys_log_deinit();
  sys_network_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
}

static void sys_manager_rental_noti_limit_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
    return;
  manager_handler.is_noti_limited_active = true;
  sys_led_write_event(SYS_LED_EVT_NOTI_RENTAL_LIMIT);
  sys_buzzer_write_event(SYS_BUZZER_EVT_RENTAL_LIMIT);
  sys_ui_noti_set(SYS_UI_NOTI_LABEL_RENTAL_LIMIT);
}

static void sys_manager_warn_debt_handler(void)
{
  if (manager_handler.is_warning_debt_active || g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
    return;
  manager_handler.is_warning_debt_active = true;
  sys_led_write_event(SYS_LED_EVT_NOTI_WARNING_DEBT);
  sys_buzzer_write_event(SYS_BUZZER_EVT_WARN_DEBT);
  sys_ui_noti_set(SYS_UI_NOTI_LABEL_WARN_ADD_FUND);
}

static void sys_manager_clear_debt_handler(void)
{
  if (!manager_handler.is_warning_debt_active)
    return;
  manager_handler.is_warning_debt_active = false;
  sys_led_write_event(SYS_LED_EVT_OFF);
  sys_buzzer_clear_event(SYS_BUZZER_EVT_WARN_DEBT);
  sys_ui_noti_clear(SYS_UI_NOTI_LABEL_WARN_ADD_FUND);
}

static void sys_manager_warn_low_balance_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
    return;
  sys_ui_noti_set(SYS_UI_NOTI_LABEL_SHOULD_ADD_FUND);
  sys_buzzer_write_event(SYS_BUZZER_EVT_LOW_BALANCE);
  // xTimerReset starts the timer if stopped and resets the countdown if running.
  bsp_timer_reset(&manager_handler.low_balance_noti_timer);
}

static void sys_manager_low_balance_noti_timer_callback(TimerHandle_t xTimer)
{
  sys_ui_noti_clear(SYS_UI_NOTI_LABEL_SHOULD_ADD_FUND);
}

static void sys_manager_help_handler(void)
{
  sys_network_publish_noti(NETWORK_NOTI_HELP, strlen(NETWORK_NOTI_HELP));
}

static void sys_manager_noti_low_batt_handler(void)
{
  sys_network_publish_noti(NETWORK_NOTI_LOW_BATT, strlen(NETWORK_NOTI_LOW_BATT));
  sys_buzzer_write_event(SYS_BUZZER_EVT_LOW_BATT);
  sys_ui_noti_set(SYS_UI_NOTI_LABEL_LOW_BATT);
}

static void sys_manager_check_lte_band_handler(void)
{
  uint32_t lte_band = bsp_sim_check_lte_band();
  LOG_INF("Current LTE band: %u", lte_band);
}

static void sys_manager_apply_danger_noti_handler(void)
{
  // If device is LOCKED, apply the lock state to the servo.
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_LOCKED
      || g_device_info.nvs_info.curr_state == DEVICE_STATE_PAUSED)
  {
    if (g_device_info.danger_noti_enabled)
    {
      bsp_servo_lock();
      LOG_DBG("Servo locked due to danger noti enabled");
    }
    else
    {
      bsp_servo_unlock();
      LOG_DBG("Servo unlocked due to danger noti disabled");
    }
  }

  // Only affects an alarm already sounding; enable/disable of future alarms is the flag itself
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_STOLEN)
  {
    return;
  }
  if (g_device_info.danger_noti_enabled)
  {
    g_device_info.danger_level = DEVICE_DANGER_LEVEL_HIGH;
    sys_buzzer_write_event(SYS_BUZZER_EVT_STOLEN);
    sys_led_write_event(SYS_LED_EVT_NOTI_DANGER);
    LOG_DBG("Danger noti resumed");
  }
  else
  {
    sys_buzzer_clear_event(SYS_BUZZER_EVT_STOLEN);
    sys_led_write_event(SYS_LED_EVT_OFF);
    LOG_DBG("Danger noti muted");
  }
}

static void sys_manager_sync_lock_handler(void)
{
  // Retry a lock actuation that was deferred while the battery was too low.
  // Runs here (not in sys_input) because the servo settle blocks ~600 ms.
  device_info_apply_lock_state();
}

/* End of file -------------------------------------------------------- */
