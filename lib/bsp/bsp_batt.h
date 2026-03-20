/**
 * @file       bsp_batt.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     Battery monitoring interface
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_BATT_H_
#define _BSP_BATT_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
// #define BATT_MONITOR_INA226
#define BATT_MONITOR_INA219

#define BSP_BATTERY_CAPACITY_MAH  (2000.0f)
#define BSP_BATT_VOLTAGE_FULL_MV  (4200)
#define BSP_BATT_VOLTAGE_EMPTY_MV (3200)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize battery monitoring (ADC).
 *
 * @return None
 */
status_function_t bsp_batt_init(void);

/**
 * @brief Read the battery voltage.
 *
 * @return Battery voltage in millivolts.
 */
float bsp_batt_read_voltage_mv(void);

/**
 * @brief Read the battery current.
 *
 * @return Battery current in milliamps.
 *        - Positive means discharging
 *        - Negative  means charging.
 */
int32_t bsp_batt_read_current_ma(void);

#endif /*End file _BSP_BATT_H_*/

/* End of file -------------------------------------------------------- */
