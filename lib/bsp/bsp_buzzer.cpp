/**
 * @file       bsp_buzzer.cpp
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-03-22
 * @author     Hai Tran
 *
 * @brief      BSP implementation for Buzzer (PWM based)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_buzzer.h"

#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"

#include <Arduino.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_buzzer, LOG_LEVEL_BSP_BUZZER);

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  bsp_buzzer_state_t state;

  // beep_long
  size_t   beep_start_ms;
  uint32_t beep_duration_ms;

  // beep_cycle
  uint8_t  cycle_total;
  uint8_t  cycle_remaining;
  uint32_t cycle_beep_ms;    // ON duration per cycle
  uint32_t cycle_period_ms;  // total period per cycle (ON + OFF)
  size_t   cycle_step_ms;    // timestamp of current step start
  bool     cycle_is_on;      // current step: ON or OFF

  bool is_initialized;
} bsp_buzzer_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_buzzer_ctx_t buzzer_handler = {
  .state            = BSP_BUZZER_STATE_IDLE,
  .beep_start_ms    = 0,
  .beep_duration_ms = 0,
  .cycle_total      = 0,
  .cycle_remaining  = 0,
  .cycle_beep_ms    = 0,
  .cycle_period_ms  = 0,
  .cycle_step_ms    = 0,
  .cycle_is_on      = false,
  .is_initialized   = false,
};

/* Private function prototypes ---------------------------------------- */
static void bsp_buzzer_set(bool on);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_buzzer_init(void)
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  buzzer_handler.state          = BSP_BUZZER_STATE_IDLE;
  buzzer_handler.is_initialized = true;

  LOG_DBG("Buzzer initialized");
  return STATUS_OK;
}

void bsp_buzzer_process(void)
{
  if (!buzzer_handler.is_initialized)
  {
    return;
  }

  size_t now = OS_GET_TICK();

  switch (buzzer_handler.state)
  {
  case BSP_BUZZER_STATE_BEEP_LONG:
  {
    if ((now - buzzer_handler.beep_start_ms) >= buzzer_handler.beep_duration_ms)
    {
      bsp_buzzer_set(false);
      buzzer_handler.state = BSP_BUZZER_STATE_IDLE;
      LOG_DBG("beep_long done");
    }
    break;
  }

  case BSP_BUZZER_STATE_BEEP_CYCLE:
  {
    if (buzzer_handler.cycle_remaining == 0)
    {
      bsp_buzzer_set(false);
      buzzer_handler.state = BSP_BUZZER_STATE_IDLE;
      LOG_DBG("beep_cycle done (%d cycles)", buzzer_handler.cycle_total);
      break;
    }

    size_t elapsed = now - buzzer_handler.cycle_step_ms;

    if (buzzer_handler.cycle_is_on)
    {
      // Currently ON — check if beep_ms elapsed
      if (elapsed >= buzzer_handler.cycle_beep_ms)
      {
        bsp_buzzer_set(false);
        buzzer_handler.cycle_is_on   = false;
        buzzer_handler.cycle_step_ms = now;
      }
    }
    else
    {
      // Currently OFF — check if rest of period elapsed
      uint32_t off_ms = buzzer_handler.cycle_period_ms - buzzer_handler.cycle_beep_ms;
      if (elapsed >= off_ms)
      {
        buzzer_handler.cycle_remaining--;
        if (buzzer_handler.cycle_remaining > 0)
        {
          bsp_buzzer_set(true);
          buzzer_handler.cycle_is_on   = true;
          buzzer_handler.cycle_step_ms = now;
        }
        else
        {
          buzzer_handler.state = BSP_BUZZER_STATE_IDLE;
          LOG_DBG("beep_cycle done (%d cycles)", buzzer_handler.cycle_total);
        }
      }
    }
    break;
  }

  case BSP_BUZZER_STATE_IDLE:
  default: break;
  }
}

status_function_t bsp_buzzer_beep_long(uint32_t duration_ms)
{
  if (!buzzer_handler.is_initialized)
  {
    return STATUS_ERROR;
  }

  buzzer_handler.beep_duration_ms = duration_ms;
  buzzer_handler.beep_start_ms    = OS_GET_TICK();
  buzzer_handler.state            = BSP_BUZZER_STATE_BEEP_LONG;

  bsp_buzzer_set(true);

  LOG_DBG("beep_long: %lu ms", duration_ms);
  return STATUS_OK;
}

void bsp_buzzer_enable(bool enable)
{
  if (!buzzer_handler.is_initialized)
  {
    return;
  }

  // Cancel any ongoing pattern
  buzzer_handler.state = BSP_BUZZER_STATE_IDLE;
  bsp_buzzer_set(enable);

  LOG_DBG("buzzer %s", enable ? "ON" : "OFF");
}

status_function_t bsp_buzzer_beep_cycle(uint8_t cycles, uint32_t beep_ms, uint32_t period_ms)
{
  if (!buzzer_handler.is_initialized)
  {
    return STATUS_ERROR;
  }

  if (cycles == 0 || beep_ms == 0 || period_ms < beep_ms)
  {
    LOG_WRN("beep_cycle: invalid params (cycles=%d beep=%lu period=%lu)", cycles, beep_ms, period_ms);
    return STATUS_ERROR;
  }

  buzzer_handler.cycle_total     = cycles;
  buzzer_handler.cycle_remaining = cycles;
  buzzer_handler.cycle_beep_ms   = beep_ms;
  buzzer_handler.cycle_period_ms = period_ms;
  buzzer_handler.cycle_step_ms   = OS_GET_TICK();
  buzzer_handler.cycle_is_on     = true;
  buzzer_handler.state           = BSP_BUZZER_STATE_BEEP_CYCLE;

  bsp_buzzer_set(true);

  LOG_DBG("beep_cycle: %d x (on=%lu ms / period=%lu ms)", cycles, beep_ms, period_ms);
  return STATUS_OK;
}

bool bsp_buzzer_is_busy(void)
{
  return (buzzer_handler.state != BSP_BUZZER_STATE_IDLE);
}

/* Private definitions ----------------------------------------------- */
static void bsp_buzzer_set(bool on)
{
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

/* End of file -------------------------------------------------------- */