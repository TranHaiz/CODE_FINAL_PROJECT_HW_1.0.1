/**
 * @file       bsp_timer.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-12
 * @author     Hai Tran
 *
 * @brief      BSP software timer definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_TIMER_H_
#define _BSP_TIMER_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "timers.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  TimerHandle_t handle;
  uint32_t      period_ms;
  bool          auto_reload;
} bsp_timer_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize a BSP timer with the specified parameters.
 * @param[in] timer Pointer to a bsp_timer_t structure to initialize.
 * @param[in] period_ms Timer period in milliseconds.
 *  @param[in] auto_reload If true, the timer will automatically reload after expiring.
 * @param[in] callback Function to call when the timer expires.
 * @return Function status.
 */
status_function_t
bsp_timer_init(bsp_timer_t *timer, uint32_t period_ms, bool auto_reload, TimerCallbackFunction_t callback);

/**
 * @brief Start a BSP timer.
 * @param[in] timer Pointer to a bsp_timer_t structure representing the timer to start.
 * @return Function status.
 */
status_function_t bsp_timer_start(bsp_timer_t *timer);

/**
 * @brief Stop a BSP timer.
 * @param[in] timer Pointer to a bsp_timer_t structure representing the timer to stop.
 * @return Function status.
 */
status_function_t bsp_timer_stop(bsp_timer_t *timer);

/**
 * @brief Reset a BSP timer (stop and restart).
 * @param[in] timer Pointer to a bsp_timer_t structure representing the timer to reset.
 * @return Function status.
 */
status_function_t bsp_timer_reset(bsp_timer_t *timer);

#endif /*End file _BSP_TIMER_H_*/

/* End of file -------------------------------------------------------- */