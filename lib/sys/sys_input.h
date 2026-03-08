/**
 * @file       sys_input.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      System Input Layer - Sensor data management
 *
 * @details    Manages input from sensors and provides unified data structure
 *             for velocity, distance, dust concentration, and compass heading
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_INPUT_H_
#define _SYS_INPUT_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
#define SYS_INPUT_UPDATE_RATE_MS (20)  // 50Hz update rate

/* Public enumerate/structure ----------------------------------------- */

/**
 * @brief Sensor input data structure
 */
typedef struct
{
  float       velocity_ms;         // Velocity in m/s
  float       velocity_kmh;        // Velocity in km/h
  float       distance_m;          // Total distance traveled in meters
  float       dust_concentration;  // Dust concentration level
  float       heading_deg;         // Compass heading in degrees (0-360)
  const char *direction_str;       // Compass direction string ("N", "NE", etc.)
  size_t      timestamp_ms;        // Timestamp of last update
} sys_input_data_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize system input with sensors
 *
 * @return none
 */
void sys_input_init(void);

/**
 * @brief Process sensor data, poll in thread loop
 */
status_function_t sys_input_process(void);

/**
 * @brief Get current sensor input data
 *
 * @param[out] data Pointer to data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_data(sys_input_data_t *data);

#endif /*End file _SYS_INPUT_H_*/

/* End of file -------------------------------------------------------- */
