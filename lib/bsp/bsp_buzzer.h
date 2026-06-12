/**
 * @file       bsp_buzzer.h
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-03-22
 * @author     Hai Tran
 *
 * @brief      BSP for Buzzer (PWM based)
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_BUZZER_H_
#define _BSP_BUZZER_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_BUZZER_STATE_IDLE = 0,
  BSP_BUZZER_STATE_BEEP_LONG,
  BSP_BUZZER_STATE_BEEP_CYCLE,
} bsp_buzzer_state_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize buzzer GPIO
 * @return status_function_t Status of operation
 */
status_function_t bsp_buzzer_init(void);

/**
 * @brief Process buzzer state machine (non-blocking)
 * @return none
 */
void bsp_buzzer_process(void);

/**
 * @brief Beep for a fixed duration (ms), non-blocking
 * @param[in] duration_ms  Beep duration in milliseconds
 * @return status_function_t Status of operation
 */
status_function_t bsp_buzzer_beep_long(uint32_t duration_ms);

/**
 * @brief Turn buzzer on or off immediately
 * @param[in] enable  True to enable buzzer, false to disable
 * @return none
 */
void bsp_buzzer_enable(bool enable);

/**
 * @brief Beep N cycles with on/off timing
 * @param[in] cycles     Number of beep cycles
 * @param[in] beep_ms    Beep ON duration per cycle (ms)
 * @param[in] period_ms  Total period per cycle (ms), must be >= beep_ms
 * @return status_function_t Status of operation
 */
status_function_t bsp_buzzer_beep_cycle(uint8_t cycles, uint32_t beep_ms, uint32_t period_ms);

/**
 * @brief Whether a timed pattern (beep_long / beep_cycle) is still playing
 * @return true if a finite pattern is in progress, false when idle
 */
bool bsp_buzzer_is_busy(void);

#endif /*End file _BSP_BUZZER_H_*/

/* End of file -------------------------------------------------------- */