/**
 * @file       bsp_timer.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-12
 * @author     Hai Tran
 *
 * @brief      BSP software timer definitions
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_timer.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

status_function_t
bsp_timer_init(bsp_timer_t *timer, uint32_t period_ms, bool auto_reload, TimerCallbackFunction_t callback)
{
  if (timer == NULL || callback == NULL)
  {
    return STATUS_ERROR;
  }

  timer->period_ms   = period_ms;
  timer->auto_reload = auto_reload;
  timer->handle =
    xTimerCreate("bsp_timer", pdMS_TO_TICKS(period_ms), auto_reload ? pdTRUE : pdFALSE, (void *) timer, callback);

  if (timer->handle == NULL)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_timer_start(bsp_timer_t *timer)
{
  if (timer == NULL || timer->handle == NULL)
  {
    return STATUS_ERROR;
  }

  if (xTimerStart(timer->handle, 0) != pdPASS)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_timer_stop(bsp_timer_t *timer)
{
  if (timer == NULL || timer->handle == NULL)
  {
    return STATUS_ERROR;
  }

  if (xTimerStop(timer->handle, 0) != pdPASS)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_timer_reset(bsp_timer_t *timer)
{
  if (timer == NULL || timer->handle == NULL)
  {
    return STATUS_ERROR;
  }

  if (xTimerReset(timer->handle, 0) != pdPASS)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */