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

#include "bsp_sim.h"
#include "sys_input.h"
#include "sys_network.h"
#include "sys_ui.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_manager, LOG_LEVEL_DBG)

/* Private enumerate/structure ---------------------------------------- */
typedef void (*sys_manager_process_handler_t)(void);
typedef struct
{
  sys_manager_event_t           current_event;
  sys_manager_process_handler_t handler[SYS_MANAGER_EVT_MAX];
} sys_manager_handler_t;

/* Private macros ----------------------------------------------------- */
#define INFO(event, func) manager_handler.handler[event] = func

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
OS_SEM_DEFINE_STATIC(sys_manager_event_sem);
static sys_manager_handler_t manager_handler;

/* Private function prototypes ---------------------------------------- */
static void sys_manager_wakeup_handler(void);
static void sys_manager_lock_handler(void);
static void sys_manager_active_handler(void);
static void sys_manager_unlocked_handler(void);
static void sys_manager_change_topic_sub_handler(void);

/* Function definitions ----------------------------------------------- */
void sys_manager_init(void)
{
  OS_SEM_CREATE(sys_manager_event_sem);
  manager_handler.current_event = SYS_MANAGER_EVT_IDLE;
  // clang-format off
  /*   Event                                |   Handlers*/
  INFO(SYS_MANAGER_EVT_WAKEUP               ,   sys_manager_wakeup_handler            );
  INFO(SYS_MANAGER_EVT_LOCKED               ,   sys_manager_lock_handler              );
  INFO(SYS_MANAGER_EVT_UNLOCKED             ,   sys_manager_unlocked_handler          );
  INFO(SYS_MANAGER_EVT_ACTIVE               ,   sys_manager_active_handler            );
  INFO(SYS_MANAGER_EVT_CHANGE_CMD_TOPIC     ,   sys_manager_change_topic_sub_handler  );
  // clang-format on
}
#undef INFO

void sys_manager_write_event(sys_manager_event_t event)
{
  if (event < SYS_MANAGER_EVT_MAX)
  {
    manager_handler.current_event = event;
    OS_SEM_GIVE(sys_manager_event_sem);
  }
}

void sys_manager_process(void)
{
  OS_SEM_TAKE(sys_manager_event_sem, OS_MAX_DELAY);

  if (manager_handler.handler[manager_handler.current_event] != nullptr)
  {
    LOG_DBG("Processed event: %d", manager_handler.current_event);
    manager_handler.handler[manager_handler.current_event]();
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_manager_wakeup_handler(void)
{
  LOG_DBG("Handling wakeup event");
}
static void sys_manager_lock_handler(void)
{
  g_device_info.state = DEVICE_STATE_LOCKED;
  sys_ui_lock();
  LOG_DBG("Device locked");
}
static void sys_manager_active_handler(void)
{
  LOG_DBG("Handling active event");
}
static void sys_manager_unlocked_handler(void)
{
  g_device_info.state = DEVICE_STATE_ACTIVE;
  sys_ui_unlock();
  LOG_DBG("Device unlocked and active");
}
static void sys_manager_change_topic_sub_handler(void)
{
  LOG_DBG("Changing MQTT sub cmd change %s to new topic: %s", g_device_info.last_mqtt_cmd_topic,
          g_device_info.mqtt_cmd_topic);
  bsp_sim_mqtt_unsub(g_device_info.last_mqtt_cmd_topic);
  bsp_sim_mqtt_sub(g_device_info.mqtt_cmd_topic, sys_network_mqtt_message_cb);
  strncpy(g_device_info.last_mqtt_cmd_topic, g_device_info.mqtt_cmd_topic, strlen(g_device_info.last_mqtt_cmd_topic));
  LOG_DBG("MQTT last cmd topic: %s", g_device_info.last_mqtt_cmd_topic);
}

/* End of file -------------------------------------------------------- */
