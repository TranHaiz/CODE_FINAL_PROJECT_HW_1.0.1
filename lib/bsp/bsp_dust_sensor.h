/**
 * @file       bsp_dust_sensor.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP for Sharp GP2Y1010AU0F Dust Sensor
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_DUST_SENSOR_H_
#define _BSP_DUST_SENSOR_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */

typedef struct
{
  uint16_t dust_density;      // Current dust ug/m^3
  uint16_t running_average;   // Running average dust density in ug/m^3
  float    baseline_voltage;  // Current baseline (zero-dust) voltage
  size_t   timestamp_ms;      // Timestamp of last reading
} bsp_dust_sensor_data_t;

typedef enum
{
  BSP_DUST_AQI_EXCELLENT = 0,  // 0-12 ug/m^3
  BSP_DUST_AQI_GOOD,           // 13-35 ug/m^3
  BSP_DUST_AQI_MODERATE,       // 36-55 ug/m^3
  BSP_DUST_AQI_POOR,           // 56-150 ug/m^3
  BSP_DUST_AQI_UNHEALTHY,      // 151-250 ug/m^3
  BSP_DUST_AQI_HAZARDOUS       // >250 ug/m^3
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
 * @param[in]  density Dust density in ug/m^3
 *
 * @return AQI level
 */
bsp_dust_aqi_level_t bsp_dust_sensor_get_aqi_level(uint16_t density);

#endif /* _BSP_DUST_SENSOR_H_ */

/* End of file -------------------------------------------------------- */
