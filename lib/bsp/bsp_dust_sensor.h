/**
 * @file       bsp_dust_sensor.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP for Sharp GP2Y1010AU0F Dust Sensor
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_DUST_SENSOR_H_
#define _BSP_DUST_SENSOR_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <GP2YDustSensor.h>

/* Public defines ----------------------------------------------------- */
#define BSP_DUST_SENSOR_DEFAULT_SAMPLES        (20)    // Default number of samples per reading
#define BSP_DUST_SENSOR_DEFAULT_AVG_COUNT      (60)    // Default running average count
#define BSP_DUST_SENSOR_DEFAULT_BASELINE       (0.4f)  // Default no-dust voltage baseline
#define BSP_DUST_SENSOR_DEFAULT_CALIBRATION    (1.0f)  // Default calibration factor
#define BSP_DUST_SENSOR_DEFAULT_UPDATE_RATE_MS (1000)  // Default update rate: 1Hz

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Dust sensor data structure with density information
 */
typedef struct
{
  uint16_t dust_density;      // Current dust density in µg/m³
  uint16_t running_average;   // Running average dust density in µg/m³
  float    baseline_voltage;  // Current baseline (zero-dust) voltage
  float    sensitivity;       // Sensor sensitivity factor
  size_t   timestamp_ms;      // Timestamp of last reading
  bool     is_valid;          // True if reading is valid
} bsp_dust_sensor_data_t;

/**
 * @brief Dust sensor callback function type
 */
typedef void (*bsp_dust_sensor_callback_t)(bsp_dust_sensor_data_t *data);

/**
 * @brief Dust sensor air quality level classification
 */
typedef enum
{
  BSP_DUST_AQI_EXCELLENT = 0,  // 0-12 µg/m³
  BSP_DUST_AQI_GOOD,           // 13-35 µg/m³
  BSP_DUST_AQI_MODERATE,       // 36-55 µg/m³
  BSP_DUST_AQI_POOR,           // 56-150 µg/m³
  BSP_DUST_AQI_UNHEALTHY,      // 151-250 µg/m³
  BSP_DUST_AQI_HAZARDOUS       // >250 µg/m³
} bsp_dust_aqi_level_t;

/**
 * @brief Dust sensor configuration structure
 */
typedef struct
{
  uint8_t  led_pin;             // LED control pin
  uint8_t  analog_pin;          // Analog input pin
  uint16_t sample_count;        // Number of samples per reading
  uint16_t avg_buffer_size;     // Running average buffer size
  float    baseline_voltage;    // Zero-dust voltage baseline
  float    calibration_factor;  // Calibration factor
  uint16_t update_rate_ms;      // Update rate in milliseconds
} bsp_dust_sensor_config_t;

/**
 * @brief Dust sensor context structure
 */
typedef struct
{
  GP2YDustSensor            *sensor;          // Pointer to GP2Y sensor object
  bsp_dust_sensor_config_t   config;          // Sensor configuration
  bsp_dust_sensor_data_t     data;            // Current sensor data
  bsp_dust_sensor_callback_t callback;        // User callback function
  size_t                     last_update_ms;  // Last update timestamp
  bool                       is_initialized;  // Initialization flag
} bsp_dust_sensor_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize dust sensor with default configuration
 *
 * @param[in] ctx     Pointer to dust sensor context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_init(bsp_dust_sensor_t *ctx);

/**
 * @brief Initialize dust sensor with custom configuration
 *
 * @param[in] ctx     Pointer to dust sensor context
 * @param[in] config  Pointer to custom configuration
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_init_with_config(bsp_dust_sensor_t *ctx, const bsp_dust_sensor_config_t *config);

/**
 * @brief Start dust sensor operation
 *
 * @param[in] ctx Pointer to dust sensor context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_begin(bsp_dust_sensor_t *ctx);

/**
 * @brief Read current dust density
 *
 * @param[in] ctx Pointer to dust sensor context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_read(bsp_dust_sensor_t *ctx);

/**
 * @brief Get current dust sensor data
 *
 * @param[in]  ctx  Pointer to dust sensor context
 * @param[out] data Pointer to data structure to fill
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_data(bsp_dust_sensor_t *ctx, bsp_dust_sensor_data_t *data);

/**
 * @brief Get current dust density value
 *
 * @param[in]  ctx     Pointer to dust sensor context
 * @param[out] density Pointer to store density value (µg/m³)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_density(bsp_dust_sensor_t *ctx, uint16_t *density);

/**
 * @brief Get running average dust density
 *
 * @param[in]  ctx     Pointer to dust sensor context
 * @param[out] average Pointer to store average value (µg/m³)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_running_average(bsp_dust_sensor_t *ctx, uint16_t *average);

/**
 * @brief Get Air Quality Index level based on dust density
 *
 * @param[in]  density Dust density in µg/m³
 * @param[out] level   Pointer to store AQI level
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_aqi_level(uint16_t density, bsp_dust_aqi_level_t *level);

/**
 * @brief Set baseline voltage (zero-dust reference)
 *
 * @param[in] ctx      Pointer to dust sensor context
 * @param[in] baseline Baseline voltage value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_set_baseline(bsp_dust_sensor_t *ctx, float baseline);

/**
 * @brief Get current baseline voltage
 *
 * @param[in]  ctx      Pointer to dust sensor context
 * @param[out] baseline Pointer to store baseline value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_baseline(bsp_dust_sensor_t *ctx, float *baseline);

/**
 * @brief Set sensor calibration factor
 *
 * @param[in] ctx    Pointer to dust sensor context
 * @param[in] factor Calibration factor value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_set_calibration(bsp_dust_sensor_t *ctx, float factor);

/**
 * @brief Set sensor sensitivity
 *
 * @param[in] ctx         Pointer to dust sensor context
 * @param[in] sensitivity Sensitivity value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_set_sensitivity(bsp_dust_sensor_t *ctx, float sensitivity);

/**
 * @brief Get sensor sensitivity
 *
 * @param[in]  ctx         Pointer to dust sensor context
 * @param[out] sensitivity Pointer to store sensitivity value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_sensitivity(bsp_dust_sensor_t *ctx, float *sensitivity);

/**
 * @brief Register callback function for periodic updates
 *
 * @param[in] ctx      Pointer to dust sensor context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_register_callback(bsp_dust_sensor_t *ctx, bsp_dust_sensor_callback_t callback);

/**
 * @brief Update dust sensor (call periodically in main loop)
 *
 * @param[in] ctx Pointer to dust sensor context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_update(bsp_dust_sensor_t *ctx);

/**
 * @brief Deinitialize dust sensor and free resources
 *
 * @param[in] ctx Pointer to dust sensor context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_deinit(bsp_dust_sensor_t *ctx);

#endif /*End file _BSP_DUST_SENSOR_H_*/

/* End of file -------------------------------------------------------- */
