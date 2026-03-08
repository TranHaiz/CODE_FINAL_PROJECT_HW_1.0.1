/**
 * @file       bsp_dust_sensor.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      BSP implementation for Sharp GP2Y1010AU0F Dust Sensor (Singleton)
 *
 * @note       This module provides low-level hardware access only.
 *             Processing should be done at system layer.
 *             Only supports single instance.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_dust_sensor.h"

#include "device_config.h"
#include "os_lib.h"

#include <Arduino.h>
#include <GP2YDustSensor.h>

/* Private defines ---------------------------------------------------- */
#define BSP_DUST_SENSOR_DEFAULT_SAMPLES   (20)
#define BSP_DUST_SENSOR_DEFAULT_AVG_COUNT (60)
#define BSP_DUST_SENSOR_DEFAULT_BASELINE  (0.4f)
#define BSP_DUST_SENSOR_DEFAULT_CALIB     (1.0f)

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Internal dust sensor context (singleton)
 */
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
static bsp_dust_sensor_ctx_t s_dust_ctx = {
  .sensor         = nullptr,
  .data           = { 0 },
  .last_update_ms = 0,
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

status_function_t bsp_dust_sensor_init(void)
{
  if (s_dust_ctx.is_initialized)
  {
    return STATUS_OK;
  }

  // Create GP2Y sensor object
  s_dust_ctx.sensor = new GP2YDustSensor(GP2YDustSensorType::GP2Y1010AU0F, DUST_SENSOR_LED_PIN, DUST_SENSOR_VO_PIN,
                                         BSP_DUST_SENSOR_DEFAULT_AVG_COUNT);

  if (s_dust_ctx.sensor == nullptr)
  {
    Serial.println("[DUST_SENSOR] ERROR: Failed to create sensor object");
    return STATUS_ERROR;
  }

  // Configure sensor
  s_dust_ctx.sensor->setBaseline(BSP_DUST_SENSOR_DEFAULT_BASELINE);
  s_dust_ctx.sensor->setCalibrationFactor(BSP_DUST_SENSOR_DEFAULT_CALIB);
  s_dust_ctx.sensor->begin();

  // Initialize data structure
  s_dust_ctx.data.dust_density     = 0;
  s_dust_ctx.data.running_average  = 0;
  s_dust_ctx.data.baseline_voltage = BSP_DUST_SENSOR_DEFAULT_BASELINE;
  s_dust_ctx.data.timestamp_ms     = 0;

  s_dust_ctx.last_update_ms = OS_GET_TICK();
  s_dust_ctx.is_initialized = true;

  Serial.printf("[DUST_SENSOR] Initialized with LED pin=%d, Analog pin=%d\n", DUST_SENSOR_LED_PIN, DUST_SENSOR_VO_PIN);

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_read(bsp_dust_sensor_data_t *data)
{
  if (data == nullptr || !s_dust_ctx.is_initialized || s_dust_ctx.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Read dust density
  uint16_t density  = s_dust_ctx.sensor->getDustDensity(BSP_DUST_SENSOR_DEFAULT_SAMPLES);
  uint16_t average  = s_dust_ctx.sensor->getRunningAverage();
  float    baseline = s_dust_ctx.sensor->getBaseline();

  // Update internal data
  s_dust_ctx.data.dust_density     = density;
  s_dust_ctx.data.running_average  = average;
  s_dust_ctx.data.baseline_voltage = baseline;
  s_dust_ctx.data.timestamp_ms     = OS_GET_TICK();
  s_dust_ctx.last_update_ms        = s_dust_ctx.data.timestamp_ms;

  // Return data to caller
  *data = s_dust_ctx.data;

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_get_aqi_level(uint16_t density, bsp_dust_aqi_level_t *level)
{
  if (level == nullptr)
  {
    return STATUS_ERROR;
  }

  if (density <= 12)
  {
    *level = BSP_DUST_AQI_EXCELLENT;
  }
  else if (density <= 35)
  {
    *level = BSP_DUST_AQI_GOOD;
  }
  else if (density <= 55)
  {
    *level = BSP_DUST_AQI_MODERATE;
  }
  else if (density <= 150)
  {
    *level = BSP_DUST_AQI_POOR;
  }
  else if (density <= 250)
  {
    *level = BSP_DUST_AQI_UNHEALTHY;
  }
  else
  {
    *level = BSP_DUST_AQI_HAZARDOUS;
  }

  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
