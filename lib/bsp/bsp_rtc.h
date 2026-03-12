/**
 * @file       bsp_rtc.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2025-08-19
 * @author     Hai Tran
 *
 * @brief      Header file for RTC BSP layer.
 *
 * @note       This BSP allows RTC initialization, set and read RTC all time data .
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef BSP_RTC_H
#define BSP_RTC_H

/* Includes ----------------------------------------------------------- */
#include "RTClib.h"
#include "common_type.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the RTC.
 *
 * @return Status function
 */
status_function_t bsp_rtc_init(void);

/**
 * @brief Set time to RTC.
 *
 * @param[inout]   rtc   Pointer to structure store information of RTC used in the project
 * @return Status function
 */
status_function_t bsp_rtc_set(timeline_t *time);

/**
 * @brief Get time from RTC.
 *
 * @return Status function
 */
status_function_t bsp_rtc_get(timeline_t *time);

#endif  // BSP_RTC_H

/* End of file -------------------------------------------------------- */
