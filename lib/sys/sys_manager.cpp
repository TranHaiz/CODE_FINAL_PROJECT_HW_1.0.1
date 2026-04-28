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
#include "bsp_buzzer.h"
#include "bsp_device.h"
#include "bsp_led.h"
#include "bsp_sdcard.h"
#include "bsp_sim.h"
#include "bsp_timer.h"
#include "cbuffer.h"
#include "device_info.h"
#include "sys_input.h"
#include "sys_network.h"
#include "sys_ui.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_manager, LOG_LEVEL_DBG)

#if (DEVICE_IDLE_MODE_ENABLED)
#define SHUTDOWN_TIMER_PERIOD_MS (30000)
#endif  // DEVICE_IDLE_MODE_ENABLED

#define DEVICE_DANGER_NOTI_INTERVAL_MS (15000)

/* Private enumerate/structure ---------------------------------------- */
typedef void (*sys_manager_process_handler_t)(void);

#define SYS_MANAGER_EVENT_QUEUE_SIZE 20

typedef struct
{
  cbuffer_t                     event_cb;
  sys_manager_process_handler_t handler[SYS_MANAGER_EVT_MAX];
  bsp_timer_t                   shutdown_timer;
  bsp_timer_t                   danger_noti_timer;
} sys_manager_handler_t;

/* Private macros ----------------------------------------------------- */
#define INFO(event, func) manager_handler.handler[event] = func

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
OS_SEM_DEFINE_STATIC(sys_manager_event_sem);
OS_MUTEX_DEFINE_STATIC(sys_manager_event_mutex);
static sys_manager_handler_t manager_handler;
static sys_manager_event_t   s_event_buffer[SYS_MANAGER_EVENT_QUEUE_SIZE];

/* Private function prototypes ---------------------------------------- */
static void sys_manager_wakeup_handler(void);
static void sys_manager_lock_handler(void);
static void sys_manager_active_handler(void);
static void sys_manager_unlocked_handler(void);
static void sys_manager_change_topic_sub_handler(void);
static void sys_manager_reboot_handler(void);
static void sys_manager_user_lock_handler(void);
static void sys_manager_user_pause_handler(void);
static void sys_manager_shutdown_timer_callback(TimerHandle_t xTimer);
static void sys_manager_shutdown_handler(void);
static void sys_manager_device_danger_handler(void);
static void sys_manager_unlock_from_network_handler(void);
static void sys_manager_lock_from_network_handler(void);
static void sys_manager_stop_rental_fail_handler(void);
static void sys_manager_stop_rental_success_handler(void);
static void sys_manager_reset_offline_data_handler(void);
static void sys_manager_danger_noti_timer_callback(TimerHandle_t xTimer);
static void sys_manager_stop_danger_noti(void);

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
  INFO(SYS_MANAGER_EVT_SHUTDOWN             ,   sys_manager_shutdown_handler            );
  INFO(SYS_MANAGER_EVT_DEVICE_DANGER        ,   sys_manager_device_danger_handler       );
  INFO(SYS_MANAGER_EVT_UNLOCK_FROM_NETWORK  ,   sys_manager_unlock_from_network_handler );
  INFO(SYS_MANAGER_EVT_LOCK_FROM_NETWORK    ,   sys_manager_lock_from_network_handler   );
  INFO(SYS_MANAGER_EVT_STOP_RENTAL_FAIL     ,   sys_manager_stop_rental_fail_handler    );
  INFO(SYS_MANAGER_EVT_STOP_RENTAL_SUCCESS  ,   sys_manager_stop_rental_success_handler );
  INFO(SYS_MANAGER_EVT_RESET_OFFLINE_DATA   ,   sys_manager_reset_offline_data_handler  );
  INFO(SYS_MANAGER_EVT_STOP_DANGER_NOTI     ,   sys_manager_stop_danger_noti            );
  // clang-format on
}
#undef INFO

void sys_manager_write_event(sys_manager_event_t event)
{
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

  OS_MUTEX_LOCK(sys_manager_event_mutex);
  size_t read = cb_read(&manager_handler.event_cb, &event_to_process, sizeof(sys_manager_event_t));
  OS_MUTEX_UNLOCK(sys_manager_event_mutex);

  if (read == sizeof(sys_manager_event_t) && event_to_process < SYS_MANAGER_EVT_MAX
      && manager_handler.handler[event_to_process] != nullptr)
  {
    LOG_DBG("Processed event: %d", event_to_process);
    manager_handler.handler[event_to_process]();
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_manager_wakeup_handler(void)
{
  LOG_DBG("Handling wakeup event");
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_wakeup();
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
}

static void sys_manager_lock_handler(void)
{
  // TODO: NOTI state to server
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
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_danger_noti();
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
    sys_ui_unlock();
    LOG_DBG("Device unlocked and active");
  }
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_danger_noti();
}

static void sys_manager_change_topic_sub_handler(void)
{
  LOG_DBG("Changing MQTT sub cmd change %s to new topic: %s", g_device_info.last_mqtt_cmd_topic,
          g_device_info.mqtt_cmd_topic);
  bsp_sim_mqtt_unsub(g_device_info.last_mqtt_cmd_topic);
  bsp_sim_mqtt_sub(g_device_info.mqtt_cmd_topic, sys_network_mqtt_message_cb);
  strncpy(g_device_info.last_mqtt_cmd_topic, g_device_info.mqtt_cmd_topic,
          sizeof(g_device_info.last_mqtt_cmd_topic) - 1);
  LOG_DBG("MQTT last cmd topic: %s", g_device_info.last_mqtt_cmd_topic);
}

static void sys_manager_reboot_handler(void)
{
  bsp_device_flash_write(&g_device_info.nvs_info);
  LOG_INF("---------- Rebooting device ----------");
  bsp_device_reboot();
}

static void sys_manager_user_lock_handler(void)
{
  LOG_DBG("Handling user lock event");
  sys_network_mqtt_publish_noti(NETWORK_NOTI_USERLOCK_PAYLOAD, strlen(NETWORK_NOTI_USERLOCK_PAYLOAD));
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_ui_lock();

#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
}

static void sys_manager_user_pause_handler(void)
{
  LOG_DBG("Handling user pause event");
  sys_network_mqtt_publish_noti(NETWORK_NOTI_USERPAUSE_PAYLOAD, strlen(NETWORK_NOTI_USERPAUSE_PAYLOAD));
  // TODO: implement pause functionality, for now just lock and send noti
  device_info_update_state(DEVICE_STATE_LOCKED);
  // device_info_update_state(DEVICE_STATE_PAUSED);
  sys_ui_lock();

#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
}

static void sys_manager_shutdown_timer_callback(TimerHandle_t xTimer)
{
  sys_manager_write_event(SYS_MANAGER_EVT_SHUTDOWN);
}

static void sys_manager_shutdown_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_IDLE)
  {
    bsp_device_flash_write(&g_device_info.nvs_info);
    device_info_update_state(DEVICE_STATE_IDLE);
    bsp_acc_enable_interrupt(BSP_ACC_INT_PIN_1);
    LOG_INF("---------- Device go to idle mode ----------");
  }
}

static void sys_manager_device_danger_handler(void)
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
    bsp_buzzer_enable(true);
    bsp_led_set(BSP_LED_COLOR_RED, BSP_LED_MODE_FLASH_FAST, 100);
    break;
  }
  case DEVICE_STATE_ACTIVE:
  {
    // TODO: Noti to server
    bsp_led_set(BSP_LED_COLOR_ORANGE, BSP_LED_MODE_PULSE, 100);
    break;
  }
  default: break;
  }
  bsp_timer_start(&manager_handler.danger_noti_timer);
}

void sys_manager_unlock_from_network_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
  {
#if (DEVICE_IDLE_MODE_ENABLED)
    bsp_timer_reset(&manager_handler.shutdown_timer);
    bsp_timer_stop(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
    device_info_update_state(DEVICE_STATE_ACTIVE);
    if (g_device_info.nvs_info.prev_state == DEVICE_STATE_IDLE)
    {
      sys_ui_wakeup();
    }
    sys_ui_unlock();
    LOG_DBG("Device unlocked and active from network");
  }
  g_device_info.danger_level = DEVICE_DANGER_LEVEL_LOW;
  sys_manager_stop_danger_noti();
  sys_network_mqtt_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
}

void sys_manager_lock_from_network_handler(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_LOCKED)
  {
    device_info_update_state(DEVICE_STATE_LOCKED);
    sys_ui_lock();
#if (DEVICE_IDLE_MODE_ENABLED)
    bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
    LOG_DBG("Device locked from network");
  }
  sys_network_mqtt_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
}

static void sys_manager_stop_rental_fail_handler(void)
{
  device_info_update_state(DEVICE_STATE_ACTIVE);
  sys_ui_warning_out_of_zone(true);
}

static void sys_manager_stop_rental_success_handler(void)
{
  sys_ui_warning_out_of_zone(false);
  device_info_update_state(DEVICE_STATE_LOCKED);
  sys_network_mqtt_publish_noti(NETWORK_DEVICE_RESP_OK_PAYLOAD, strlen(NETWORK_DEVICE_RESP_OK_PAYLOAD));
#if (DEVICE_IDLE_MODE_ENABLED)
  bsp_timer_start(&manager_handler.shutdown_timer);
#endif  // DEVICE_IDLE_MODE_ENABLED
}

static void sys_manager_reset_offline_data_handler(void)
{
  status_function_t ret = bsp_sdcard_delete(SD_OFFLINE_LOG_PATH);
  if (ret != STATUS_OK)
  {
    LOG_ERR("Failed to delete offline data");
  }
  OS_DELAY_MS(1000);
  bsp_device_reboot();
}

static void sys_manager_danger_noti_timer_callback(TimerHandle_t xTimer)
{
  sys_manager_write_event(SYS_MANAGER_EVT_STOP_DANGER_NOTI);
}

static void sys_manager_stop_danger_noti(void)
{
  bsp_led_off();
  sys_ui_wakeup();
  bsp_buzzer_enable(false);
}

/* End of file -------------------------------------------------------- */
