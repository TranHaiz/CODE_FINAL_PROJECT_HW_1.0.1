/**
 * @file       sys_buzzer.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-25
 * @author     Hai Tran
 *
 * @brief     System service buzzer control with priority-based event queue
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_BUZZER_H_
#define _SYS_BUZZER_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_buzzer.h"
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
// Order = priority: lower index = higher priority. OFF is a sentinel that
// clears low-priority notification events (mirrors sys_led).
typedef enum
{
  SYS_BUZZER_EVT_OFF = 0,
  SYS_BUZZER_EVT_STOLEN,
  SYS_BUZZER_EVT_RENTAL_LIMIT,
  SYS_BUZZER_EVT_LOW_BATT,
  SYS_BUZZER_EVT_WARN_DEBT,
  SYS_BUZZER_EVT_LOW_BALANCE,
  SYS_BUZZER_EVT_STARTUP,
  SYS_BUZZER_EVT_MAX
} sys_buzzer_evt_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the system buzzer (bsp_buzzer + event table)
 */
void sys_buzzer_init(void);

/**
 * @brief Activate a buzzer event (priority resolves which pattern plays)
 * @param[in] event The event to write
 */
void sys_buzzer_write_event(sys_buzzer_evt_t event);

/**
 * @brief Deactivate a buzzer event
 * @param[in] event The event to clear
 */
void sys_buzzer_clear_event(sys_buzzer_evt_t event);

/**
 * @brief Process buzzer events, dispatch highest-priority pattern, drive bsp
 */
void sys_buzzer_process(void);

#endif /*End file _SYS_BUZZER_H_*/

/* End of file -------------------------------------------------------- */
