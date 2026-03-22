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
LOG_MODULE_REGISTER(bsp_buzzer, LOG_LEVEL_INFO);
#define BSP_BUZZER_PWM_CHANNEL (0)
#define BSP_BUZZER_PWM_FREQ    (4000)
#define BSP_BUZZER_PWM_RES     (8)
#define BSP_BUZZER_DUTY_OFF    (0)
#define BSP_BUZZER_DUTY_MAX    (255)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bool is_buzzer_initialized = false;

/* Private function prototypes ---------------------------------------- */
static void buzzer_pwm_set(uint8_t duty);

/* Function definitions ----------------------------------------------- */
status_function_t bsp_buzzer_init(void)
{
  if (is_buzzer_initialized)
    return STATUS_OK;

  ledcSetup(BSP_BUZZER_PWM_CHANNEL, BSP_BUZZER_PWM_FREQ, BSP_BUZZER_PWM_RES);
  ledcAttachPin(BUZZER_PIN, BSP_BUZZER_PWM_CHANNEL);
  buzzer_pwm_set(BSP_BUZZER_DUTY_OFF);
  is_buzzer_initialized = true;
  return STATUS_OK;
}

status_function_t bsp_buzzer_beep(uint8_t volume_percent, uint8_t beep_count, uint16_t delay_ms)
{
  if (!is_buzzer_initialized)
    bsp_buzzer_init();
  uint8_t duty = (volume_percent > 100 ? BSP_BUZZER_DUTY_MAX : (volume_percent * BSP_BUZZER_DUTY_MAX) / 100);
  for (uint8_t i = 0; i < beep_count; ++i)
  {
    buzzer_pwm_set(duty);
    OS_DELAY_MS(100);
    buzzer_pwm_set(BSP_BUZZER_DUTY_OFF);
    if (i < beep_count - 1)
      OS_DELAY_MS(delay_ms);
  }
  return STATUS_OK;
}

status_function_t bsp_buzzer_on(uint8_t volume_percent)
{
  if (!is_buzzer_initialized)
    bsp_buzzer_init();
  uint8_t duty = (volume_percent > 100 ? BSP_BUZZER_DUTY_MAX : (volume_percent * BSP_BUZZER_DUTY_MAX) / 100);
  buzzer_pwm_set(duty);
  return STATUS_OK;
}

status_function_t bsp_buzzer_off(void)
{
  if (!is_buzzer_initialized)
    return STATUS_ERROR;
  buzzer_pwm_set(BSP_BUZZER_DUTY_OFF);
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
static void buzzer_pwm_set(uint8_t duty)
{
  ledcWrite(BSP_BUZZER_PWM_CHANNEL, duty);
}

/* End of file -------------------------------------------------------- */
