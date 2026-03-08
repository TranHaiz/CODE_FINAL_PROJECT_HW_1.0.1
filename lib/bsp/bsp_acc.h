/**
 * @file       bsp_acc.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      BSP for LSM6DS3 Accelerometer/Gyroscope Sensor (Singleton)
 *
 * @note       This module provides low-level hardware access only.
 *             Processing should be done at system layer.
 *             Only supports single instance.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_ACC_H_
#define _BSP_ACC_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
#define BSP_ACC_GRAVITY_MS2 (9.806f)  // Earth gravity in m/s²

/* Public enumerate/structure ----------------------------------------- */

/**
 * @brief Accelerometer raw data structure
 */
typedef struct
{
  float accel_x;  // Acceleration X-axis in g
  float accel_y;  // Acceleration Y-axis in g
  float accel_z;  // Acceleration Z-axis in g
  float gyro_x;   // Angular velocity X-axis in dps
  float gyro_y;   // Angular velocity Y-axis in dps
  float gyro_z;   // Angular velocity Z-axis in dps
  float temp_c;   // Temperature in Celsius
} bsp_acc_raw_data_t;

/**
 * @brief Accelerometer processed data structure
 */
typedef struct
{
  float  accel_magnitude;  // Total acceleration magnitude in g
  size_t timestamp_ms;     // Timestamp of last reading
  bool   is_valid;         // True if reading is valid
} bsp_acc_data_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize accelerometer sensor
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_init(void);

/**
 * @brief Read raw accelerometer/gyroscope data
 *
 * @param[out] data    Pointer to raw data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_read_raw(bsp_acc_raw_data_t *data);

/**
 * @brief Get processed accelerometer data (magnitude)
 *
 * @param[out] data    Pointer to data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_get_data(bsp_acc_data_t *data);

#endif /* _BSP_ACC_H_ */

/* End of file -------------------------------------------------------- */
