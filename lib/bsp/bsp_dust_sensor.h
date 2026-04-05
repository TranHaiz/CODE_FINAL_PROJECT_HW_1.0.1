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
  uint16_t dust_density;
  uint16_t running_average;
  float    baseline_voltage;
  size_t   timestamp_ms;
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
 * @brief  Initialize dust sensor with warmup and baseline calibration
 *
 * @note   Blocks for ~30s during warmup. Call once at startup.
 *
 * @return status_function_t
 */
status_function_t bsp_dust_sensor_init(void);

/**
 * @brief  Read dust density
 *
 * @param[out] data  Pointer to output data structure
 *
 * @return status_function_t
 */
status_function_t bsp_dust_sensor_read(bsp_dust_sensor_data_t *data);

/**
 * @brief  Map density value to AQI level
 *
 * @param[in]  density  Dust density in ug/m^3
 *
 * @return bsp_dust_aqi_level_t
 */
bsp_dust_aqi_level_t bsp_dust_sensor_get_aqi_level(uint16_t density);

#endif /* _BSP_DUST_SENSOR_H_ */

/* End of file -------------------------------------------------------- */
