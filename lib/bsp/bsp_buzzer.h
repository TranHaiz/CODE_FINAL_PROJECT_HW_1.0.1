/**
 * @file       bsp_buzzer.h
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-03-22
 * @author     Hai Tran
 *
 * @brief      BSP for Buzzer (PWM based)
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_BUZZER_H_
#define _BSP_BUZZER_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize the buzzer (PWM setup)
 *
 * @return status_function_t
 */
status_function_t bsp_buzzer_init(void);

/**
 * @brief Beep the buzzer (blocking)
 *
 * @param[in] volume_percent  Volume (0-100%)
 * @param[in] beep_count      Number of beeps
 * @param[in] delay_ms        Delay between beeps (ms)
 *
 * @return status_function_t
 */
status_function_t bsp_buzzer_beep(uint8_t volume_percent, uint8_t beep_count, uint16_t delay_ms);

/**
 * @brief Turn on the buzzer with specified volume (non-blocking)
 *
 * @param[in] volume_percent  Volume (0-100%)
 *
 * @return status_function_t
 */
status_function_t bsp_buzzer_on(uint8_t volume_percent);

/**
 * @brief Turn off the buzzer
 *
 * @return status_function_t
 */
status_function_t bsp_buzzer_off(void);

#endif /*End file _BSP_BUZZER_H_*/

/* End of file -------------------------------------------------------- */
