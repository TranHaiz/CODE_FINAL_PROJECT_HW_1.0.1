/**
 * @file       bsp_servo.cpp
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-06-14
 * @author     Hai Tran
 *
 * @brief      BSP implementation for SG90 servo (ESP32Servo library)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_servo.h"

#include "device_config.h"
#include "log_service.h"

#include <ESP32Servo.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_servo, LOG_LEVEL_BSP_SERVO);

#define SERVO_FREQ_HZ (50)    // SG90: 50 Hz
#define SERVO_MIN_US  (500)   // pulse width at 0 deg
#define SERVO_MAX_US  (2400)  // pulse width at 180 deg
#define SERVO_MAX_DEG (180)
#define SERVO_TIMER   (0)     // dedicate LEDC timer 0; backlight uses ch7/timer3

#define SERVO_SELFTEST_STEP_DEG (10)   // sweep increment
#define SERVO_SELFTEST_STEP_MS  (40)   // dwell per step
#define SERVO_SELFTEST_HOLD_MS  (500)  // hold at lock/unlock ends

#define SERVO_SETTLE_MS (600)  // time to reach target before de-energizing

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  SERVO_POS_UNKNOWN = 0,  // force the first command to actuate
  SERVO_POS_LOCKED,
  SERVO_POS_UNLOCKED
} servo_pos_t;

typedef struct
{
  Servo       servo;
  bool        is_initialized;
  servo_pos_t pos;
} bsp_servo_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_servo_ctx_t servo_handler;

/* Private function prototypes ---------------------------------------- */
static void servo_drive(uint8_t angle_deg);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_servo_init(void)
{
  ESP32PWM::allocateTimer(SERVO_TIMER);
  servo_handler.servo.setPeriodHertz(SERVO_FREQ_HZ);
  servo_handler.servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  if (!servo_handler.servo.attached())
  {
    LOG_ERR("Servo attach failed on pin %d", SERVO_PIN);
    return STATUS_ERROR;
  }

  servo_handler.is_initialized = true;
  servo_handler.pos            = SERVO_POS_UNKNOWN;
  // Do not move here: leave the (self-holding) mechanism where it is and stay
  // de-energized. The lock state is synced later once power is known to be safe.
  servo_handler.servo.release();

  LOG_DBG("Servo initialized on pin %d", SERVO_PIN);
  return STATUS_OK;
}

status_function_t bsp_servo_set_angle(uint8_t angle_deg)
{
  if (!servo_handler.is_initialized)
  {
    return STATUS_ERROR;
  }

  if (angle_deg > SERVO_MAX_DEG)
  {
    angle_deg = SERVO_MAX_DEG;
  }

  servo_handler.servo.write(angle_deg);

  LOG_DBG("angle=%u deg", angle_deg);
  return STATUS_OK;
}

status_function_t bsp_servo_lock(void)
{
  if (!servo_handler.is_initialized)
  {
    return STATUS_ERROR;
  }
  if (servo_handler.pos == SERVO_POS_LOCKED)
  {
    return STATUS_OK;  // already locked: skip to avoid needless re-drive/jitter
  }

  LOG_DBG("Lock");
  servo_drive(SERVO_LOCK);
  servo_handler.pos = SERVO_POS_LOCKED;
  return STATUS_OK;
}

status_function_t bsp_servo_unlock(void)
{
  if (!servo_handler.is_initialized)
  {
    return STATUS_ERROR;
  }
  if (servo_handler.pos == SERVO_POS_UNLOCKED)
  {
    return STATUS_OK;  // already unlocked: skip to avoid needless re-drive/jitter
  }

  LOG_DBG("Unlock");
  servo_drive(SERVO_UNLOCK);
  servo_handler.pos = SERVO_POS_UNLOCKED;
  return STATUS_OK;
}

status_function_t bsp_servo_selftest(void)
{
  if (!servo_handler.is_initialized)
  {
    LOG_ERR("Selftest skipped: not initialized");
    return STATUS_ERROR;
  }

  LOG_DBG("Selftest start");
  servo_handler.pos = SERVO_POS_UNKNOWN;  // force the lock/unlock below to actuate

  // Sweep up 0 -> 180, then down 180 -> 0
  for (int angle = SERVO_LOCK; angle <= SERVO_UNLOCK; angle += SERVO_SELFTEST_STEP_DEG)
  {
    bsp_servo_set_angle(angle);
    delay(SERVO_SELFTEST_STEP_MS);
  }
  for (int angle = SERVO_UNLOCK; angle >= SERVO_LOCK; angle -= SERVO_SELFTEST_STEP_DEG)
  {
    bsp_servo_set_angle(angle);
    delay(SERVO_SELFTEST_STEP_MS);
  }

  // Exercise the named lock/unlock positions
  bsp_servo_unlock();
  delay(SERVO_SELFTEST_HOLD_MS);
  bsp_servo_lock();
  delay(SERVO_SELFTEST_HOLD_MS);

  LOG_DBG("Selftest done");
  return STATUS_OK;
}

void bsp_servo_deinit(void)
{
  if (!servo_handler.is_initialized)
  {
    return;
  }

  servo_handler.servo.detach();
  servo_handler.is_initialized = false;
  servo_handler.pos            = SERVO_POS_UNKNOWN;

  LOG_DBG("Servo deinitialized");
}

/* Private definitions ----------------------------------------------- */
// Move to a target, hold long enough to settle, then de-energize. The mechanism
// holds position mechanically, so cutting the PWM stops the idle buzz/jitter and
// the holding current that collapses a weak battery.
static void servo_drive(uint8_t angle_deg)
{
  bsp_servo_set_angle(angle_deg);
  delay(SERVO_SETTLE_MS);
  servo_handler.servo.release();
}

/* End of file -------------------------------------------------------- */
