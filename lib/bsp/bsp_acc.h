/**
 * @file       bsp_acc.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      BSP for LSM6DS3 Accelerometer/Gyroscope Sensor
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_ACC_H_
#define _BSP_ACC_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
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

typedef struct
{
  float  accel_magnitude;  // Total acceleration magnitude in g
  size_t timestamp_ms;     // Timestamp of last reading
  bool   is_valid;         // True if reading is valid
} bsp_acc_data_t;

typedef enum
{
  BSP_ACC_INT_PIN_1 = 0,
  BSP_ACC_INT_PIN_2
} bsp_acc_int_pin_t;
typedef enum
{
  BSP_ACC_INT_ACCEL_DATA_READY = 0,
  BSP_ACC_INT_GYRO_DATA_READY,
  BSP_ACC_INT_MOTION_DETECT,
  BSP_ACC_INT_FREE_FALL,
  BSP_ACC_INT_DOUBLE_TAP,
  BSP_ACC_INT_SINGLE_TAP
} bsp_acc_int_source_t;

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
status_function_t bsp_acc_get_raw_data(bsp_acc_raw_data_t *data);

/**
 * @brief Get processed accelerometer data (magnitude)
 *
 * @param[out] data    Pointer to data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_get_data(bsp_acc_data_t *data);

/**
 * @brief Configure interrupt on specified pin
 *
 * @param[in] pin     Interrupt pin to configure (INT1 or INT2)
 * @param[in] source  Interrupt event source
 *
 * @return status_function_t Status of operation
 *
 * @note Only one event can be routed to each pin at a time.
 *       Configuring again will replace the previous configuration.
 */
status_function_t bsp_acc_config_interrupt(bsp_acc_int_pin_t pin, bsp_acc_int_source_t source);

/**
 * @brief Enable interrupt on specified pin
 *
 * @param[in] pin     Interrupt pin to enable (INT1 or INT2)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_enable_interrupt(bsp_acc_int_pin_t pin);

/**
 * @brief Disable interrupt on specified pin
 *
 * @param[in] pin     Interrupt pin to disable (INT1 or INT2)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_disable_interrupt(bsp_acc_int_pin_t pin);

#endif /* _BSP_ACC_H_ */

/* End of file -------------------------------------------------------- */
