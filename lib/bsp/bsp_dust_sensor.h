/**
 * @file       bsp_dust_sensor.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      BSP for Sharp GP2Y1010AU0F Dust Sensor (Singleton)
 *
 * @note       This module provides low-level hardware access only.
 *             Processing should be done at system layer.
 *             Only supports single instance.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_DUST_SENSOR_H_
#define _BSP_DUST_SENSOR_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */

/**
 * @brief Dust sensor data structure
 */
typedef struct
{
  uint16_t dust_density;      // Current dust density in µg/m³
  uint16_t running_average;   // Running average dust density in µg/m³
  float    baseline_voltage;  // Current baseline (zero-dust) voltage
  size_t   timestamp_ms;      // Timestamp of last reading
  bool     is_valid;          // True if reading is valid
} bsp_dust_sensor_data_t;

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

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize dust sensor
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_init(void);

/**
 * @brief Read dust sensor data
 *
 * @param[out] data Pointer to data structure to fill
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_read(bsp_dust_sensor_data_t *data);

/**
 * @brief Get air quality level from density value
 *
 * @param[in]  density Dust density in µg/m³
 * @param[out] level   Pointer to AQI level
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_dust_sensor_get_aqi_level(uint16_t density, bsp_dust_aqi_level_t *level);

#endif /* _BSP_DUST_SENSOR_H_ */

/* End of file -------------------------------------------------------- */
