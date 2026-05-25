/**
 * @file       sys_manager.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-31
 * @author     Hai Tran
 *
 * @brief      Manage system state and device information
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_MANAGER_H_
#define _SYS_MANAGER_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_MANAGER_EVT_IDLE = 0,
  SYS_MANAGER_EVT_LOCKED,
  SYS_MANAGER_EVT_UNLOCKED,
  SYS_MANAGER_EVT_LOCK_FROM_NETWORK,
  SYS_MANAGER_EVT_UNLOCK_FROM_NETWORK,
  SYS_MANAGER_EVT_ACTIVE,
  SYS_MANAGER_EVT_WAKEUP,
  SYS_MANAGER_EVT_CHANGE_CMD_TOPIC,
  SYS_MANAGER_EVT_REBOOT,
  SYS_MANAGER_EVT_USER_LOCK,
  SYS_MANAGER_EVT_USER_PAUSE,
  SYS_MANAGER_EVT_SHUTDOWN,
  SYS_MANAGER_EVT_DEVICE_STOLEN,
  SYS_MANAGER_EVT_START_RENTAL,
  SYS_MANAGER_EVT_STOP_RENTAL_FAIL,
  SYS_MANAGER_EVT_STOP_RENTAL_SUCCESS,
  SYS_MANAGER_EVT_RESET_OFFLINE_DATA,
  SYS_MANAGER_EVT_STOP_STOLEN_NOTI,
  SYS_MANAGER_EVT_STOLEN_TIMEOUT,
  SYS_MANAGER_EVT_FLUSH_LOG,
  SYS_MANAGER_RENTAL_NOTI_LIMIT,
  SYS_MANAGER_EVT_WARN_LOW_BALANCE,
  SYS_MANAGER_EVT_WARN_DEBT,
  SYS_MANAGER_EVT_CLEAR_DEBT,
  SYS_MANAGER_EVT_HELP,
  SYS_MANAGER_EVT_MAX
} sys_manager_event_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize system manager
 *
 * @return none
 */
void sys_manager_init(void);

/**
 * @brief Write event function, call to submit events
 */
void sys_manager_write_event(sys_manager_event_t event);

/**
 * @brief Process function, call in thread loop
 */
void sys_manager_process(void);

#endif /*End file _SYS_MANAGER_H_*/

/* End of file -------------------------------------------------------- */
