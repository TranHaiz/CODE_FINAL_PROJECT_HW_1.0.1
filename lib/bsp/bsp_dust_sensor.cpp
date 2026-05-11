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

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_dust_sensor, LOG_LEVEL_BSP_DUST_SENSOR);

#define BSP_DUST_SENSOR_ADC_RES        (12)
#define BSP_DUST_SENSOR_ADC_MAX        (4095.0f)
#define BSP_DUST_SENSOR_VCC            (3.3f)
#define BSP_DUST_SENSOR_CAL_FACTOR     (1000.0f)
#define BSP_DUST_SENSOR_SAMPLES        (10)
#define BSP_DUST_SENSOR_BASELINE_SAMPS (50)
#define BSP_DUST_SENSOR_WARMUP_MS      (5000u)
#define BSP_DUST_SENSOR_T_ON_US        (280u)  // LED on → wait → sample (datasheet)
#define BSP_DUST_SENSOR_T_HOLD_US      (40u)
#define BSP_DUST_SENSOR_T_CYCLE_MS     (10u)
#define BSP_DUST_SENSOR_MAX_UGM3       (600.0f)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  float  baseline_adc;
  size_t last_update_ms;
  bool   is_initialized;
} bsp_dust_sensor_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_dust_sensor_ctx_t dust_ctx = {
  .baseline_adc   = 0.0f,
  .last_update_ms = 0,
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
static int   bsp_dust_sensor_read_once(void);
static float bsp_dust_sensor_calibrate_baseline(void);

/* Function definitions ----------------------------------------------- */
status_function_t bsp_dust_sensor_init(void)
{
  if (dust_ctx.is_initialized)
  {
    return STATUS_OK;
  }

  pinMode(DUST_SENSOR_LED_PIN, OUTPUT);
  digitalWrite(DUST_SENSOR_LED_PIN, LOW);  // LED off

  analogReadResolution(BSP_DUST_SENSOR_ADC_RES);
  analogSetPinAttenuation(DUST_SENSOR_AOOUT_PIN, ADC_11db);
  OS_DELAY_MS(100);

  // Warmup
  LOG_DBG("Warming up dust sensor (%dms)...", BSP_DUST_SENSOR_WARMUP_MS);
  size_t warmup_end = OS_GET_TICK() + BSP_DUST_SENSOR_WARMUP_MS;
  while (OS_GET_TICK() < warmup_end)
  {
    bsp_dust_sensor_read_once();
    OS_YIELD();
  }
  LOG_DBG("Warmup done");

  // Calibrate baseline
  LOG_DBG("Calibrating baseline (%d samples)...", BSP_DUST_SENSOR_BASELINE_SAMPS);
  dust_ctx.baseline_adc = bsp_dust_sensor_calibrate_baseline();
  LOG_DBG("Baseline ADC = %.2f (%.4fV)", dust_ctx.baseline_adc,
          dust_ctx.baseline_adc * (BSP_DUST_SENSOR_VCC / BSP_DUST_SENSOR_ADC_MAX));

  dust_ctx.last_update_ms = OS_GET_TICK();
  dust_ctx.is_initialized = true;

  return STATUS_OK;
}

status_function_t bsp_dust_sensor_read(bsp_dust_sensor_data_t *data)
{
  if (data == nullptr || !dust_ctx.is_initialized)
  {
    return STATUS_ERROR;
  }

  float sum = 0.0f;
  for (uint8_t i = 0; i < BSP_DUST_SENSOR_SAMPLES; i++)
  {
    sum += (float) bsp_dust_sensor_read_once();
  }
  float adc_avg = sum / (float) BSP_DUST_SENSOR_SAMPLES;

  float delta_adc = adc_avg - dust_ctx.baseline_adc;
  if (delta_adc < 0.0f)
  {
    delta_adc = 0.0f;
  }

  float delta_v = delta_adc * (BSP_DUST_SENSOR_VCC / BSP_DUST_SENSOR_ADC_MAX);
  float density = delta_v * BSP_DUST_SENSOR_CAL_FACTOR;
  if (density > BSP_DUST_SENSOR_MAX_UGM3)
  {
    density = BSP_DUST_SENSOR_MAX_UGM3;
  }

  // Fill output
  data->dust_density     = (uint16_t) density;
  data->running_average  = data->dust_density;
  data->baseline_voltage = dust_ctx.baseline_adc * (BSP_DUST_SENSOR_VCC / BSP_DUST_SENSOR_ADC_MAX);
  data->timestamp_ms     = OS_GET_TICK();

  dust_ctx.last_update_ms = data->timestamp_ms;

  LOG_DBG("adc_avg=%.1f delta_adc=%.1f delta_v=%.4fV density=%d ug/m3", adc_avg, delta_adc, delta_v,
          data->dust_density);

  return STATUS_OK;
}

bsp_dust_aqi_level_t bsp_dust_sensor_get_aqi_level(float density)
{
  if (density <= 12.0f)
  {
    return BSP_DUST_AQI_EXCELLENT;
  }
  else if (density <= 35.0f)
  {
    return BSP_DUST_AQI_GOOD;
  }
  else if (density <= 55.0f)
  {
    return BSP_DUST_AQI_MODERATE;
  }
  else if (density <= 150.0f)
  {
    return BSP_DUST_AQI_POOR;
  }
  else if (density <= 250.0f)
  {
    return BSP_DUST_AQI_UNHEALTHY;
  }
  else
  {
    return BSP_DUST_AQI_HAZARDOUS;
  }
}

/* Private definitions ----------------------------------------------- */
static int bsp_dust_sensor_read_once(void)
{
  digitalWrite(DUST_SENSOR_LED_PIN, HIGH);  // LED on (active HIGH - Waveshare board)
  delayMicroseconds(BSP_DUST_SENSOR_T_ON_US);
  int adc = analogRead(DUST_SENSOR_AOOUT_PIN);
  delayMicroseconds(BSP_DUST_SENSOR_T_HOLD_US);
  digitalWrite(DUST_SENSOR_LED_PIN, LOW);  // LED off
  OS_DELAY_MS(BSP_DUST_SENSOR_T_CYCLE_MS);
  return adc;
}

static float bsp_dust_sensor_calibrate_baseline(void)
{
  long sum = 0;
  for (uint8_t i = 0; i < BSP_DUST_SENSOR_BASELINE_SAMPS; i++)
  {
    sum += bsp_dust_sensor_read_once();
  }
  return (float) sum / (float) BSP_DUST_SENSOR_BASELINE_SAMPS;
}

/* End of file -------------------------------------------------------- */