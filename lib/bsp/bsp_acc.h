/**
 * @file       bsp_acc.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
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

#include <SparkFunLSM6DS3.h>
#include <Wire.h>

/* Public defines ----------------------------------------------------- */
#define BSP_ACC_DEFAULT_I2C_ADDR          (0x6B)
#define BSP_ACC_DEFAULT_SDA_PIN           (4)
#define BSP_ACC_DEFAULT_SCL_PIN           (5)
#define BSP_ACC_DEFAULT_CALIBRATION_COUNT (200)
#define BSP_ACC_DEFAULT_SAMPLE_DELAY_MS   (5)
#define BSP_ACC_DEFAULT_ALPHA             (0.15f)   // Low-pass filter coefficient
#define BSP_ACC_DEFAULT_ACCEL_THRESHOLD   (0.15f)   // Acceleration threshold in m/s²
#define BSP_ACC_DEFAULT_FRICTION_FACTOR   (0.98f)   // Simulated friction factor
#define BSP_ACC_DEFAULT_UPDATE_RATE_MS    (20)      // 50Hz sampling rate
#define BSP_ACC_GRAVITY_MS2               (9.806f)  // Earth gravity in m/s²

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
  float    accel_magnitude;  // Total acceleration magnitude in g
  float    accel_filtered;   // Filtered acceleration (gravity removed)
  float    accel_ms2;        // Acceleration in m/s²
  float    velocity_ms;      // Velocity in m/s
  float    velocity_kmh;     // Velocity in km/h
  uint32_t timestamp_ms;     // Timestamp of last reading
  bool     is_valid;         // True if reading is valid
  bool     is_calibrated;    // True if sensor is calibrated
} bsp_acc_data_t;

/**
 * @brief Accelerometer callback function type
 */
typedef void (*bsp_acc_callback_t)(bsp_acc_data_t *data);

/**
 * @brief Accelerometer raw data callback function type
 */
typedef void (*bsp_acc_raw_callback_t)(bsp_acc_raw_data_t *data);

/**
 * @brief Accelerometer configuration structure
 */
typedef struct
{
  uint8_t  i2c_addr;             // I2C address (0x6A or 0x6B)
  uint8_t  sda_pin;              // I2C SDA pin
  uint8_t  scl_pin;              // I2C SCL pin
  uint16_t calibration_count;    // Number of samples for calibration
  uint16_t sample_delay_ms;      // Delay between calibration samples
  float    alpha;                // Low-pass filter coefficient (0.0 - 1.0)
  float    accel_threshold_ms2;  // Acceleration threshold in m/s²
  float    friction_factor;      // Friction simulation factor (0.0 - 1.0)
  uint16_t update_rate_ms;       // Update rate in milliseconds
} bsp_acc_config_t;

/**
 * @brief Accelerometer context structure
 */
typedef struct
{
  LSM6DS3               *sensor;            // Pointer to LSM6DS3 sensor object
  bsp_acc_config_t       config;            // Sensor configuration
  bsp_acc_data_t         data;              // Processed sensor data
  bsp_acc_raw_data_t     raw_data;          // Raw sensor data
  bsp_acc_callback_t     callback;          // User callback for processed data
  bsp_acc_raw_callback_t raw_callback;      // User callback for raw data
  float                  offset_magnitude;  // Gravity offset from calibration
  uint32_t               last_update_us;    // Last update timestamp (microseconds)
  uint32_t               last_update_ms;    // Last update timestamp (milliseconds)
  bool                   is_initialized;    // Initialization flag
  bool                   is_calibrated;     // Calibration flag
} bsp_acc_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize accelerometer with default configuration
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_init(bsp_acc_t *ctx);

/**
 * @brief Initialize accelerometer with custom configuration
 *
 * @param[in] ctx     Pointer to accelerometer context
 * @param[in] config  Pointer to configuration structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_init_with_config(bsp_acc_t *ctx, const bsp_acc_config_t *config);

/**
 * @brief Start the accelerometer sensor
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_begin(bsp_acc_t *ctx);

/**
 * @brief Calibrate the accelerometer (remove gravity offset)
 * @note  Sensor should be stationary during calibration
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_calibrate(bsp_acc_t *ctx);

/**
 * @brief Update sensor readings and process data
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_update(bsp_acc_t *ctx);

/**
 * @brief Read raw accelerometer data
 *
 * @param[in]  ctx     Pointer to accelerometer context
 * @param[out] data    Pointer to raw data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_read_raw(bsp_acc_t *ctx, bsp_acc_raw_data_t *data);

/**
 * @brief Get processed accelerometer data
 *
 * @param[in]  ctx     Pointer to accelerometer context
 * @param[out] data    Pointer to data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_get_data(bsp_acc_t *ctx, bsp_acc_data_t *data);

/**
 * @brief Get current velocity in m/s
 *
 * @param[in]  ctx         Pointer to accelerometer context
 * @param[out] velocity_ms Pointer to velocity value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_get_velocity_ms(bsp_acc_t *ctx, float *velocity_ms);

/**
 * @brief Get current velocity in km/h
 *
 * @param[in]  ctx          Pointer to accelerometer context
 * @param[out] velocity_kmh Pointer to velocity value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_get_velocity_kmh(bsp_acc_t *ctx, float *velocity_kmh);

/**
 * @brief Get current acceleration in m/s²
 *
 * @param[in]  ctx       Pointer to accelerometer context
 * @param[out] accel_ms2 Pointer to acceleration value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_get_accel_ms2(bsp_acc_t *ctx, float *accel_ms2);

/**
 * @brief Reset velocity to zero
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_reset_velocity(bsp_acc_t *ctx);

/**
 * @brief Register callback for processed data
 *
 * @param[in] ctx      Pointer to accelerometer context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_register_callback(bsp_acc_t *ctx, bsp_acc_callback_t callback);

/**
 * @brief Register callback for raw data
 *
 * @param[in] ctx      Pointer to accelerometer context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_register_raw_callback(bsp_acc_t *ctx, bsp_acc_raw_callback_t callback);

/**
 * @brief Set low-pass filter coefficient
 *
 * @param[in] ctx   Pointer to accelerometer context
 * @param[in] alpha Filter coefficient (0.0 - 1.0, lower = smoother)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_set_filter_alpha(bsp_acc_t *ctx, float alpha);

/**
 * @brief Set acceleration threshold
 *
 * @param[in] ctx          Pointer to accelerometer context
 * @param[in] threshold_ms2 Threshold in m/s²
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_set_threshold(bsp_acc_t *ctx, float threshold_ms2);

/**
 * @brief Check if sensor is calibrated
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return true if calibrated, false otherwise
 */
bool bsp_acc_is_calibrated(bsp_acc_t *ctx);

/**
 * @brief Scan I2C bus for devices (debug helper)
 *
 * @param[in] sda_pin SDA pin number
 * @param[in] scl_pin SCL pin number
 *
 * @return int8_t Found I2C address, or -1 if not found
 */
int8_t bsp_acc_scan_i2c(uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Deinitialize accelerometer and free resources
 *
 * @param[in] ctx     Pointer to accelerometer context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_acc_deinit(bsp_acc_t *ctx);

#endif /*End file _BSP_ACC_H_*/

/* End of file -------------------------------------------------------- */
