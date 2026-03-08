/**
 * @file       sys_input.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      System Input Layer - GPS/INS Sensor Fusion for Velocity Estimation
 *
 * @details    Implements GPS/INS integration using complementary filter approach:
 *             - GPS provides absolute velocity reference at low frequency
 *             - IMU provides high-frequency velocity updates via integration
 *             - Compass provides heading reference for direction
 *             - Complementary filter fuses GPS and INS data
 *
 *             Reference: GPS/INS Integration methodology
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_INPUT_H_
#define _SYS_INPUT_H_

/* Includes ----------------------------------------------------------- */
#include "bsp_acc.h"
#include "bsp_compass.h"
#include "bsp_gps.h"
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
#define SYS_INPUT_DEFAULT_FILTER_ALPHA    (0.15f)   // Low-pass filter for acceleration
#define SYS_INPUT_DEFAULT_COMP_FILTER_K   (0.98f)   // Complementary filter GPS weight
#define SYS_INPUT_DEFAULT_ACCEL_THRESHOLD (0.15f)   // Min acceleration to update velocity (m/s²)
#define SYS_INPUT_DEFAULT_FRICTION_FACTOR (0.98f)   // Friction simulation factor
#define SYS_INPUT_DEFAULT_UPDATE_RATE_MS  (20)      // 50Hz update rate
#define SYS_INPUT_GPS_VALID_TIMEOUT_MS    (2000)    // GPS data validity timeout
#define SYS_INPUT_INS_DECAY_FACTOR        (0.995f)  // INS velocity decay when no GPS
#define SYS_INPUT_GRAVITY_MS2             (9.806f)  // Earth gravity in m/s²

/* Public enumerate/structure ----------------------------------------- */

/**
 * @brief Sensor fusion mode
 */
typedef enum
{
  SYS_INPUT_MODE_GPS_ONLY = 0,    // Use GPS velocity only
  SYS_INPUT_MODE_INS_ONLY,        // Use INS (accelerometer) velocity only
  SYS_INPUT_MODE_GPS_INS_FUSION,  // GPS/INS complementary filter fusion (default)
  SYS_INPUT_MODE_AUTO             // Auto switch between GPS and INS based on availability
} sys_input_fusion_mode_t;

/**
 * @brief Sensor status flags
 */
typedef struct
{
  bool gps_valid;       // GPS has valid fix
  bool gps_recent;      // GPS data is recent (within timeout)
  bool imu_calibrated;  // IMU is calibrated
  bool compass_valid;   // Compass data is valid
} sys_input_sensor_status_t;

/**
 * @brief Velocity data structure
 */
typedef struct
{
  float  velocity_ms;     // Fused velocity in m/s
  float  velocity_kmh;    // Fused velocity in km/h
  float  velocity_gps;    // GPS-derived velocity in m/s
  float  velocity_ins;    // INS-derived velocity in m/s
  float  heading_deg;     // Heading in degrees (0-360)
  float  heading_rad;     // Heading in radians
  float  accel_ms2;       // Current acceleration in m/s²
  float  accel_filtered;  // Filtered acceleration
  size_t timestamp_ms;    // Timestamp of last update
  bool   is_valid;        // True if data is valid
} sys_input_velocity_data_t;

/**
 * @brief Position data structure (from GPS)
 */
typedef struct
{
  double latitude;      // Latitude in degrees
  double longitude;     // Longitude in degrees
  double altitude;      // Altitude in meters
  double course_deg;    // Course/bearing in degrees
  size_t satellites;    // Number of GPS satellites
  double hdop;          // Horizontal dilution of precision
  size_t timestamp_ms;  // Timestamp of last update
  bool   is_valid;      // True if position is valid
} sys_input_position_data_t;

/**
 * @brief Complete sensor input data structure
 */
typedef struct
{
  sys_input_velocity_data_t velocity;  // Velocity information
  sys_input_position_data_t position;  // Position information
  sys_input_sensor_status_t status;    // Sensor status flags
  sys_input_fusion_mode_t   mode;      // Current fusion mode
  bsp_acc_raw_data_t        imu_raw;   // Raw IMU data
  bsp_compass_data_t        compass;   // Compass data
  bsp_gps_data_t            gps;       // GPS data
} sys_input_data_t;

/**
 * @brief Callback function type for velocity updates
 */
typedef void (*sys_input_velocity_callback_t)(sys_input_velocity_data_t *data);

/**
 * @brief Callback function type for position updates
 */
typedef void (*sys_input_position_callback_t)(sys_input_position_data_t *data);

/**
 * @brief Configuration structure for sys_input
 */
typedef struct
{
  sys_input_fusion_mode_t fusion_mode;          // Fusion mode selection
  float                   filter_alpha;         // Low-pass filter coefficient
  float                   comp_filter_k;        // Complementary filter GPS weight
  float                   accel_threshold_ms2;  // Acceleration threshold
  float                   friction_factor;      // Friction simulation factor
  uint16_t                update_rate_ms;       // Update rate in milliseconds
  size_t                  gps_timeout_ms;       // GPS validity timeout
} sys_input_config_t;

/**
 * @brief Context structure for sys_input
 */
typedef struct
{
  sys_input_config_t            config;            // Configuration
  sys_input_data_t              data;              // Current sensor data
  sys_input_velocity_callback_t vel_callback;      // Velocity callback
  sys_input_position_callback_t pos_callback;      // Position callback
  bsp_acc_t                    *acc_ctx;           // Accelerometer context
  bsp_compass_t                *compass_ctx;       // Compass context
  float                         offset_magnitude;  // Gravity offset
  float                         accel_filtered;    // Filtered acceleration
  size_t                        last_update_us;    // Last update timestamp (us)
  size_t                        last_update_ms;    // Last update timestamp (ms)
  size_t                        last_gps_ms;       // Last valid GPS timestamp
  bool                          is_initialized;    // Initialization flag
  bool                          is_calibrated;     // Calibration flag
} sys_input_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize sys_input with default configuration
 *
 * @param[in] ctx         Pointer to sys_input context
 * @param[in] acc_ctx     Pointer to accelerometer context
 * @param[in] compass_ctx Pointer to compass context (can be NULL)
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_init(sys_input_t *ctx, bsp_acc_t *acc_ctx, bsp_compass_t *compass_ctx);

/**
 * @brief Initialize sys_input with custom configuration
 *
 * @param[in] ctx         Pointer to sys_input context
 * @param[in] acc_ctx     Pointer to accelerometer context
 * @param[in] compass_ctx Pointer to compass context (can be NULL)
 * @param[in] config      Pointer to configuration structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_init_with_config(sys_input_t              *ctx,
                                             bsp_acc_t                *acc_ctx,
                                             bsp_compass_t            *compass_ctx,
                                             const sys_input_config_t *config);

/**
 * @brief Calibrate the sensor input system (calibrate IMU gravity offset)
 * @note  Sensor should be stationary during calibration
 *
 * @param[in] ctx Pointer to sys_input context
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_calibrate(sys_input_t *ctx);

/**
 * @brief Update all sensor readings and perform sensor fusion
 * @note  Should be called at regular intervals (e.g., 50Hz)
 *
 * @param[in] ctx Pointer to sys_input context
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_update(sys_input_t *ctx);

/**
 * @brief Get fused velocity data
 *
 * @param[in]  ctx  Pointer to sys_input context
 * @param[out] data Pointer to velocity data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_velocity(sys_input_t *ctx, sys_input_velocity_data_t *data);

/**
 * @brief Get position data
 *
 * @param[in]  ctx  Pointer to sys_input context
 * @param[out] data Pointer to position data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_position(sys_input_t *ctx, sys_input_position_data_t *data);

/**
 * @brief Get complete sensor input data
 *
 * @param[in]  ctx  Pointer to sys_input context
 * @param[out] data Pointer to complete data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_data(sys_input_t *ctx, sys_input_data_t *data);

/**
 * @brief Get current velocity in m/s
 *
 * @param[in]  ctx         Pointer to sys_input context
 * @param[out] velocity_ms Pointer to velocity value
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_velocity_ms(sys_input_t *ctx, float *velocity_ms);

/**
 * @brief Get current velocity in km/h
 *
 * @param[in]  ctx          Pointer to sys_input context
 * @param[out] velocity_kmh Pointer to velocity value
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_velocity_kmh(sys_input_t *ctx, float *velocity_kmh);

/**
 * @brief Get current heading in degrees
 *
 * @param[in]  ctx         Pointer to sys_input context
 * @param[out] heading_deg Pointer to heading value
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_heading(sys_input_t *ctx, float *heading_deg);

/**
 * @brief Get current acceleration in m/s²
 *
 * @param[in]  ctx       Pointer to sys_input context
 * @param[out] accel_ms2 Pointer to acceleration value
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_accel(sys_input_t *ctx, float *accel_ms2);

/**
 * @brief Get sensor status
 *
 * @param[in]  ctx    Pointer to sys_input context
 * @param[out] status Pointer to status structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_get_status(sys_input_t *ctx, sys_input_sensor_status_t *status);

/**
 * @brief Reset velocity to zero
 *
 * @param[in] ctx Pointer to sys_input context
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_reset_velocity(sys_input_t *ctx);

/**
 * @brief Set fusion mode
 *
 * @param[in] ctx  Pointer to sys_input context
 * @param[in] mode Fusion mode to set
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_set_mode(sys_input_t *ctx, sys_input_fusion_mode_t mode);

/**
 * @brief Set complementary filter GPS weight
 *
 * @param[in] ctx Pointer to sys_input context
 * @param[in] k   Filter weight (0.0 = full INS, 1.0 = full GPS)
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_set_comp_filter_k(sys_input_t *ctx, float k);

/**
 * @brief Set low-pass filter coefficient
 *
 * @param[in] ctx   Pointer to sys_input context
 * @param[in] alpha Filter coefficient (0.0 - 1.0)
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_set_filter_alpha(sys_input_t *ctx, float alpha);

/**
 * @brief Set acceleration threshold
 *
 * @param[in] ctx          Pointer to sys_input context
 * @param[in] threshold_ms2 Threshold in m/s²
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_set_threshold(sys_input_t *ctx, float threshold_ms2);

/**
 * @brief Register callback for velocity updates
 *
 * @param[in] ctx      Pointer to sys_input context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_register_velocity_callback(sys_input_t *ctx, sys_input_velocity_callback_t callback);

/**
 * @brief Register callback for position updates
 *
 * @param[in] ctx      Pointer to sys_input context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_register_position_callback(sys_input_t *ctx, sys_input_position_callback_t callback);

/**
 * @brief Check if system is calibrated
 *
 * @param[in] ctx Pointer to sys_input context
 *
 * @return true if calibrated, false otherwise
 */
bool sys_input_is_calibrated(sys_input_t *ctx);

/**
 * @brief Check if GPS is available and recent
 *
 * @param[in] ctx Pointer to sys_input context
 *
 * @return true if GPS is available, false otherwise
 */
bool sys_input_is_gps_available(sys_input_t *ctx);

/**
 * @brief Deinitialize sys_input
 *
 * @param[in] ctx Pointer to sys_input context
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_input_deinit(sys_input_t *ctx);

#endif /*End file _SYS_INPUT_H_*/

/* End of file -------------------------------------------------------- */
