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
#include "log_service.h"
#include "os_lib.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_MANAGER_EVT_IDLE = 0,
  SYS_MANAGER_EVT_LOCKED,
  SYS_MANAGER_EVT_UNLOCKED,
  SYS_MANAGER_EVT_ACTIVE,
  SYS_MANAGER_EVT_WAKEUP,
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
