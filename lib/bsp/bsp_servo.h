/**
 * @file       bsp_servo.h
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-06-14
 * @author     Hai Tran
 *
 * @brief      BSP for SG90 servo
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_SERVO_H_
#define _BSP_SERVO_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "device_config.h"

#include <stdint.h>

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize servo PWM on SERVO_PIN
 * @return status_function_t Status of operation
 */
status_function_t bsp_servo_init(void);

/**
 * @brief Drive the servo to a target angle
 * @param[in] angle_deg  Target angle in degrees (0..180), clamped if out of range
 * @return status_function_t Status of operation
 */
status_function_t bsp_servo_set_angle(uint8_t angle_deg);

/**
 * @brief Drive the servo to the locked position (SERVO_LOCK)
 * @return status_function_t Status of operation
 */
status_function_t bsp_servo_lock(void);

/**
 * @brief Drive the servo to the unlocked position (SERVO_UNLOCK)
 * @return status_function_t Status of operation
 */
status_function_t bsp_servo_unlock(void);

/**
 * @brief Blocking self-test: sweep the full range and exercise lock/unlock
 * @return status_function_t Status of operation
 */
status_function_t bsp_servo_selftest(void);

/**
 * @brief Release the servo PWM and free the GPIO
 * @return none
 */
void bsp_servo_deinit(void);

#endif /*End file _BSP_SERVO_H_*/

/* End of file -------------------------------------------------------- */
