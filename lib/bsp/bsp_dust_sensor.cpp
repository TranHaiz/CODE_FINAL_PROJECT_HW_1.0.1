/**
 * @file       bsp_dust_sensor.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP implementation for Sharp GP2Y1010AU0F Dust Sensor
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_dust_sensor.h"

#include "device_pin_conf.h"
#include "os_lib.h"

#include <Arduino.h>

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

status_function_t bsp_dust_sensor_init(bsp_dust_sensor_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  // Set default configuration
  bsp_dust_sensor_config_t default_config = {
    .led_pin            = DUST_SENSOR_LED_PIN,
    .analog_pin         = DUST_SENSOR_VO_PIN,
    .sample_count       = BSP_DUST_SENSOR_DEFAULT_SAMPLES,
    .avg_buffer_size    = BSP_DUST_SENSOR_DEFAULT_AVG_COUNT,
    .baseline_voltage   = BSP_DUST_SENSOR_DEFAULT_BASELINE,
    .calibration_factor = BSP_DUST_SENSOR_DEFAULT_CALIBRATION,
    .update_rate_ms     = BSP_DUST_SENSOR_DEFAULT_UPDATE_RATE_MS,
  };

  return bsp_dust_sensor_init_with_config(ctx, &default_config);
}

status_function_t bsp_dust_sensor_init_with_config(bsp_dust_sensor_t *ctx, const bsp_dust_sensor_config_t *config)
{
  if (ctx == nullptr || config == nullptr)
  {
    return STATUS_ERROR;
  }

  // Copy configuration
  ctx->config = *config;

  // Create GP2Y sensor object
  ctx->sensor = new GP2YDustSensor(GP2YDustSensorType::GP2Y1010AU0F, ctx->config.led_pin, ctx->config.analog_pin,
                                   ctx->config.avg_buffer_size);

  if (ctx->sensor == nullptr)
  {
    printf("[DUST_SENSOR] ERROR: Failed to create sensor object\n");
    return STATUS_ERROR;
  }

  // Initialize data structure
  ctx->data.dust_density     = 0;
  ctx->data.running_average  = 0;
  ctx->data.baseline_voltage = config->baseline_voltage;
  ctx->data.sensitivity      = BSP_DUST_SENSOR_DEFAULT_CALIBRATION;
  ctx->data.timestamp_ms     = 0;
  ctx->data.is_valid         = false;

  // Initialize state
  ctx->callback       = nullptr;
  ctx->last_update_ms = 0;
  ctx->is_initialized = false;

  printf("[DUST_SENSOR] Initialized with LED pin=%d, Analog pin=%d\n", config->led_pin, config->analog_pin);

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_begin(bsp_dust_sensor_t *ctx)
{
  if (ctx == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Configure sensor with baseline and calibration
  ctx->sensor->setBaseline(ctx->config.baseline_voltage);
  ctx->sensor->setCalibrationFactor(ctx->config.calibration_factor);

  // Start sensor
  ctx->sensor->begin();

  ctx->is_initialized = true;
  ctx->last_update_ms = OS_GET_TICK();

  printf("[DUST_SENSOR] Started - Baseline=%.2fV, Calibration=%.2f\n", ctx->config.baseline_voltage,
         ctx->config.calibration_factor);

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_read(bsp_dust_sensor_t *ctx)
{
  if (ctx == nullptr || ctx->sensor == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  // Read dust density
  uint16_t density  = ctx->sensor->getDustDensity(ctx->config.sample_count);
  uint16_t average  = ctx->sensor->getRunningAverage();
  float    baseline = ctx->sensor->getBaseline();

  // Update data structure
  ctx->data.dust_density     = density;
  ctx->data.running_average  = average;
  ctx->data.baseline_voltage = baseline;
  ctx->data.timestamp_ms     = OS_GET_TICK();
  ctx->data.is_valid         = true;

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_get_data(bsp_dust_sensor_t *ctx, bsp_dust_sensor_data_t *data)
{
  if (ctx == nullptr || data == nullptr)
  {
    return STATUS_ERROR;
  }

  *data = ctx->data;
  return STATUS_OK;
}

status_function_t bsp_dust_sensor_get_density(bsp_dust_sensor_t *ctx, uint16_t *density)
{
  if (ctx == nullptr || density == nullptr)
  {
    return STATUS_ERROR;
  }

  *density = ctx->data.dust_density;
  return STATUS_OK;
}

status_function_t bsp_dust_sensor_get_running_average(bsp_dust_sensor_t *ctx, uint16_t *average)
{
  if (ctx == nullptr || average == nullptr)
  {
    return STATUS_ERROR;
  }

  *average = ctx->data.running_average;
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

status_function_t bsp_dust_sensor_set_baseline(bsp_dust_sensor_t *ctx, float baseline)
{
  if (ctx == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->sensor->setBaseline(baseline);
  ctx->config.baseline_voltage = baseline;
  ctx->data.baseline_voltage   = baseline;

  printf("[DUST_SENSOR] Baseline set to %.2fV\n", baseline);

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_get_baseline(bsp_dust_sensor_t *ctx, float *baseline)
{
  if (ctx == nullptr || baseline == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  *baseline = ctx->sensor->getBaseline();
  return STATUS_OK;
}

status_function_t bsp_dust_sensor_set_calibration(bsp_dust_sensor_t *ctx, float factor)
{
  if (ctx == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->sensor->setCalibrationFactor(factor);
  ctx->config.calibration_factor = factor;

  printf("[DUST_SENSOR] Calibration factor set to %.2f\n", factor);

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_set_sensitivity(bsp_dust_sensor_t *ctx, float sensitivity)
{
  if (ctx == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->sensor->setSensitivity(sensitivity);
  ctx->data.sensitivity = sensitivity;

  printf("[DUST_SENSOR] Sensitivity set to %.2f\n", sensitivity);

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_get_sensitivity(bsp_dust_sensor_t *ctx, float *sensitivity)
{
  if (ctx == nullptr || sensitivity == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  *sensitivity = ctx->sensor->getSensitivity();
  return STATUS_OK;
}

status_function_t bsp_dust_sensor_register_callback(bsp_dust_sensor_t *ctx, bsp_dust_sensor_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->callback = callback;
  printf("[DUST_SENSOR] Callback registered\n");

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_update(bsp_dust_sensor_t *ctx)
{
  if (ctx == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  size_t now = OS_GET_TICK();

  // Check if it's time to update
  if (now - ctx->last_update_ms >= ctx->config.update_rate_ms)
  {
    ctx->last_update_ms = now;

    // Read new data
    status_function_t status = bsp_dust_sensor_read(ctx);
    if (status != STATUS_OK)
    {
      return status;
    }

    // Call user callback if registered
    if (ctx->callback != nullptr)
    {
      ctx->callback(&ctx->data);
    }
  }

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_deinit(bsp_dust_sensor_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (ctx->sensor != nullptr)
  {
    delete ctx->sensor;
    ctx->sensor = nullptr;
  }

  ctx->is_initialized = false;
  printf("[DUST_SENSOR] Deinitialized\n");

  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
