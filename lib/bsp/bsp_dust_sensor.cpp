/**
 * @file       bsp_dust_sensor.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP implementation for Sharp GP2Y1010AU0F Dust Sensor
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_dust_sensor.h"

#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"

#include <Arduino.h>
#include <GP2YDustSensor.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_dust_sensor, LOG_LEVEL_ERROR);
#define BSP_DUST_SENSOR_SAMPLES   (30)
#define BSP_DUST_SENSOR_AVG_COUNT (90)
#define BSP_DUST_SENSOR_BASELINE  (0.4f)
#define BSP_DUST_SENSOR_CALIB     (1.0f)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  GP2YDustSensor        *sensor;
  bsp_dust_sensor_data_t data;
  size_t                 last_update_ms;
  bool                   is_initialized;
} bsp_dust_sensor_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_dust_sensor_ctx_t dust_handler = {
  .sensor         = nullptr,
  .data           = { 0 },
  .last_update_ms = 0,
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
status_function_t bsp_dust_sensor_init(void)
{
  if (dust_handler.is_initialized)
  {
    return STATUS_OK;
  }

  dust_handler.sensor = new GP2YDustSensor(GP2YDustSensorType::GP2Y1010AU0F, DUST_SENSOR_LED_PIN, DUST_SENSOR_VO_PIN,
                                           BSP_DUST_SENSOR_AVG_COUNT);

  if (dust_handler.sensor == nullptr)
  {
    LOG_ERR("Failed to create sensor object");
    return STATUS_ERROR;
  }

  dust_handler.sensor->setBaseline(BSP_DUST_SENSOR_BASELINE);
  dust_handler.sensor->setCalibrationFactor(BSP_DUST_SENSOR_CALIB);
  dust_handler.sensor->begin();

  // Initialize data structure
  dust_handler.data.dust_density     = 0;
  dust_handler.data.running_average  = 0;
  dust_handler.data.baseline_voltage = BSP_DUST_SENSOR_BASELINE;
  dust_handler.data.timestamp_ms     = 0;

  dust_handler.last_update_ms = OS_GET_TICK();
  dust_handler.is_initialized = true;

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_read(bsp_dust_sensor_data_t *data)
{
  if (data == nullptr || !dust_handler.is_initialized || dust_handler.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Read dust density
  uint16_t density  = dust_handler.sensor->getDustDensity(BSP_DUST_SENSOR_SAMPLES);
  uint16_t average  = dust_handler.sensor->getRunningAverage();
  float    baseline = dust_handler.sensor->getBaseline();

  // Update internal data
  dust_handler.data.dust_density     = density;
  dust_handler.data.running_average  = average;
  dust_handler.data.baseline_voltage = baseline;
  dust_handler.data.timestamp_ms     = OS_GET_TICK();
  dust_handler.last_update_ms        = dust_handler.data.timestamp_ms;

  // Return data to caller
  *data = dust_handler.data;

  return STATUS_OK;
}

bsp_dust_aqi_level_t bsp_dust_sensor_get_aqi_level(uint16_t density)
{
  if (density <= 12)
  {
    return BSP_DUST_AQI_EXCELLENT;
  }
  else if (density <= 35)
  {
    return BSP_DUST_AQI_GOOD;
  }
  else if (density <= 55)
  {
    return BSP_DUST_AQI_MODERATE;
  }
  else if (density <= 150)
  {
    return BSP_DUST_AQI_POOR;
  }
  else if (density <= 250)
  {
    return BSP_DUST_AQI_UNHEALTHY;
  }
  else
  {
    return BSP_DUST_AQI_HAZARDOUS;
  }
}

/* End of file -------------------------------------------------------- */
