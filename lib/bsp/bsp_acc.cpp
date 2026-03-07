/**
 * @file       bsp_acc.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP implementation for LSM6DS3 Accelerometer/Gyroscope Sensor
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_acc.h"

#include "device_pin_conf.h"
#include "os_lib.h"

#include <Arduino.h>
#include <math.h>

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static float bsp_acc_calculate_magnitude(float x, float y, float z);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_acc_init(bsp_acc_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  // Set default configuration
  bsp_acc_config_t default_config = {
    .i2c_addr            = BSP_ACC_DEFAULT_I2C_ADDR,
    .sda_pin             = BSP_ACC_DEFAULT_SDA_PIN,
    .scl_pin             = BSP_ACC_DEFAULT_SCL_PIN,
    .calibration_count   = BSP_ACC_DEFAULT_CALIBRATION_COUNT,
    .sample_delay_ms     = BSP_ACC_DEFAULT_SAMPLE_DELAY_MS,
    .alpha               = BSP_ACC_DEFAULT_ALPHA,
    .accel_threshold_ms2 = BSP_ACC_DEFAULT_ACCEL_THRESHOLD,
    .friction_factor     = BSP_ACC_DEFAULT_FRICTION_FACTOR,
    .update_rate_ms      = BSP_ACC_DEFAULT_UPDATE_RATE_MS,
  };

  return bsp_acc_init_with_config(ctx, &default_config);
}

status_function_t bsp_acc_init_with_config(bsp_acc_t *ctx, const bsp_acc_config_t *config)
{
  if (ctx == nullptr || config == nullptr)
  {
    return STATUS_ERROR;
  }

  // Copy configuration
  ctx->config = *config;

  // Initialize I2C with custom pins
  Wire.begin(config->sda_pin, config->scl_pin);

  // Create LSM6DS3 sensor object
  ctx->sensor = new LSM6DS3(I2C_MODE, config->i2c_addr);

  if (ctx->sensor == nullptr)
  {
    Serial.println("[ACC] ERROR: Failed to create sensor object");
    return STATUS_ERROR;
  }

  // Initialize data structures
  ctx->data.accel_magnitude = 0.0f;
  ctx->data.accel_filtered  = 0.0f;
  ctx->data.accel_ms2       = 0.0f;
  ctx->data.velocity_ms     = 0.0f;
  ctx->data.velocity_kmh    = 0.0f;
  ctx->data.timestamp_ms    = 0;
  ctx->data.is_valid        = false;
  ctx->data.is_calibrated   = false;

  ctx->raw_data.accel_x = 0.0f;
  ctx->raw_data.accel_y = 0.0f;
  ctx->raw_data.accel_z = 0.0f;
  ctx->raw_data.gyro_x  = 0.0f;
  ctx->raw_data.gyro_y  = 0.0f;
  ctx->raw_data.gyro_z  = 0.0f;
  ctx->raw_data.temp_c  = 0.0f;

  // Initialize state
  ctx->callback         = nullptr;
  ctx->raw_callback     = nullptr;
  ctx->offset_magnitude = 0.0f;
  ctx->last_update_us   = 0;
  ctx->last_update_ms   = 0;
  ctx->is_initialized   = false;
  ctx->is_calibrated    = false;

  Serial.printf("[ACC] Initialized with I2C addr=0x%02X, SDA=%d, SCL=%d\n", config->i2c_addr, config->sda_pin,
                config->scl_pin);

  return STATUS_OK;
}

status_function_t bsp_acc_begin(bsp_acc_t *ctx)
{
  if (ctx == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Try to start sensor with current address
  uint8_t result = ctx->sensor->begin();

  // If failed, try alternate I2C address
  if (result != 0)
  {
    uint8_t alt_addr = (ctx->config.i2c_addr == 0x6B) ? 0x6A : 0x6B;
    Serial.printf("[ACC] Failed with 0x%02X, trying 0x%02X...\n", ctx->config.i2c_addr, alt_addr);

    // Create new sensor object with alternate address
    delete ctx->sensor;
    ctx->sensor = new LSM6DS3(I2C_MODE, alt_addr);

    if (ctx->sensor == nullptr || ctx->sensor->begin() != 0)
    {
      Serial.println("[ACC] ERROR: Failed to start sensor on both addresses");
      Serial.println("[ACC] Please check: 1) Wiring 2) I2C address 3) Power supply");
      return STATUS_ERROR;
    }

    ctx->config.i2c_addr = alt_addr;
    Serial.printf("[ACC] Connected on alternate address 0x%02X\n", alt_addr);
  }

  ctx->is_initialized = true;
  ctx->last_update_us = micros();
  ctx->last_update_ms = OS_GET_TICK();

  Serial.println("[ACC] Sensor started successfully");

  return STATUS_OK;
}

status_function_t bsp_acc_calibrate(bsp_acc_t *ctx)
{
  if (ctx == nullptr || ctx->sensor == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  Serial.println("[ACC] Starting calibration... Keep sensor stationary!");

  float sum_magnitude = 0.0f;

  for (uint16_t i = 0; i < ctx->config.calibration_count; i++)
  {
    float x = ctx->sensor->readFloatAccelX();
    float y = ctx->sensor->readFloatAccelY();
    float z = ctx->sensor->readFloatAccelZ();

    sum_magnitude += bsp_acc_calculate_magnitude(x, y, z);

    delay(ctx->config.sample_delay_ms);
  }

  ctx->offset_magnitude   = sum_magnitude / (float) ctx->config.calibration_count;
  ctx->is_calibrated      = true;
  ctx->data.is_calibrated = true;
  ctx->last_update_us     = micros();

  Serial.printf("[ACC] Calibration complete. Gravity offset: %.4f g\n", ctx->offset_magnitude);

  return STATUS_OK;
}

status_function_t bsp_acc_update(bsp_acc_t *ctx)
{
  if (ctx == nullptr || ctx->sensor == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  // Calculate time delta
  uint32_t current_time_us = micros();
  float    dt              = (current_time_us - ctx->last_update_us) / 1000000.0f;
  ctx->last_update_us      = current_time_us;

  // Read raw accelerometer data
  ctx->raw_data.accel_x = ctx->sensor->readFloatAccelX();
  ctx->raw_data.accel_y = ctx->sensor->readFloatAccelY();
  ctx->raw_data.accel_z = ctx->sensor->readFloatAccelZ();

  // Read gyroscope data
  ctx->raw_data.gyro_x = ctx->sensor->readFloatGyroX();
  ctx->raw_data.gyro_y = ctx->sensor->readFloatGyroY();
  ctx->raw_data.gyro_z = ctx->sensor->readFloatGyroZ();

  // Read temperature
  ctx->raw_data.temp_c = ctx->sensor->readTempC();

  // Calculate current acceleration magnitude
  float current_magnitude =
    bsp_acc_calculate_magnitude(ctx->raw_data.accel_x, ctx->raw_data.accel_y, ctx->raw_data.accel_z);

  ctx->data.accel_magnitude = current_magnitude;

  // Process data only if calibrated
  if (ctx->is_calibrated)
  {
    // Remove gravity offset
    float accel_clear = current_magnitude - ctx->offset_magnitude;

    // Apply low-pass filter
    ctx->data.accel_filtered = ctx->config.alpha * accel_clear + (1.0f - ctx->config.alpha) * ctx->data.accel_filtered;

    // Convert to m/s²
    ctx->data.accel_ms2 = ctx->data.accel_filtered * BSP_ACC_GRAVITY_MS2;

    // Integrate to calculate velocity
    if (fabsf(ctx->data.accel_ms2) > ctx->config.accel_threshold_ms2)
    {
      ctx->data.velocity_ms += ctx->data.accel_ms2 * dt;
    }
    else
    {
      // Apply friction when acceleration is below threshold
      ctx->data.velocity_ms *= ctx->config.friction_factor;
    }

    // Ensure velocity is not negative
    if (ctx->data.velocity_ms < 0.0f)
    {
      ctx->data.velocity_ms = 0.0f;
    }

    // Convert to km/h
    ctx->data.velocity_kmh = ctx->data.velocity_ms * 3.6f;
  }

  // Update timestamp and validity
  ctx->data.timestamp_ms = OS_GET_TICK();
  ctx->data.is_valid     = true;

  // Call callbacks if registered
  if (ctx->callback != nullptr)
  {
    ctx->callback(&ctx->data);
  }

  if (ctx->raw_callback != nullptr)
  {
    ctx->raw_callback(&ctx->raw_data);
  }

  return STATUS_OK;
}

status_function_t bsp_acc_read_raw(bsp_acc_t *ctx, bsp_acc_raw_data_t *data)
{
  if (ctx == nullptr || data == nullptr || ctx->sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Read all raw data
  data->accel_x = ctx->sensor->readFloatAccelX();
  data->accel_y = ctx->sensor->readFloatAccelY();
  data->accel_z = ctx->sensor->readFloatAccelZ();
  data->gyro_x  = ctx->sensor->readFloatGyroX();
  data->gyro_y  = ctx->sensor->readFloatGyroY();
  data->gyro_z  = ctx->sensor->readFloatGyroZ();
  data->temp_c  = ctx->sensor->readTempC();

  return STATUS_OK;
}

status_function_t bsp_acc_get_data(bsp_acc_t *ctx, bsp_acc_data_t *data)
{
  if (ctx == nullptr || data == nullptr)
  {
    return STATUS_ERROR;
  }

  *data = ctx->data;
  return STATUS_OK;
}

status_function_t bsp_acc_get_velocity_ms(bsp_acc_t *ctx, float *velocity_ms)
{
  if (ctx == nullptr || velocity_ms == nullptr)
  {
    return STATUS_ERROR;
  }

  *velocity_ms = ctx->data.velocity_ms;
  return STATUS_OK;
}

status_function_t bsp_acc_get_velocity_kmh(bsp_acc_t *ctx, float *velocity_kmh)
{
  if (ctx == nullptr || velocity_kmh == nullptr)
  {
    return STATUS_ERROR;
  }

  *velocity_kmh = ctx->data.velocity_kmh;
  return STATUS_OK;
}

status_function_t bsp_acc_get_accel_ms2(bsp_acc_t *ctx, float *accel_ms2)
{
  if (ctx == nullptr || accel_ms2 == nullptr)
  {
    return STATUS_ERROR;
  }

  *accel_ms2 = ctx->data.accel_ms2;
  return STATUS_OK;
}

status_function_t bsp_acc_reset_velocity(bsp_acc_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->data.velocity_ms  = 0.0f;
  ctx->data.velocity_kmh = 0.0f;

  Serial.println("[ACC] Velocity reset to zero");
  return STATUS_OK;
}

status_function_t bsp_acc_register_callback(bsp_acc_t *ctx, bsp_acc_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->callback = callback;
  return STATUS_OK;
}

status_function_t bsp_acc_register_raw_callback(bsp_acc_t *ctx, bsp_acc_raw_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->raw_callback = callback;
  return STATUS_OK;
}

status_function_t bsp_acc_set_filter_alpha(bsp_acc_t *ctx, float alpha)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (alpha < 0.0f || alpha > 1.0f)
  {
    Serial.println("[ACC] WARNING: Alpha should be between 0.0 and 1.0");
    return STATUS_ERROR;
  }

  ctx->config.alpha = alpha;
  Serial.printf("[ACC] Filter alpha set to %.2f\n", alpha);
  return STATUS_OK;
}

status_function_t bsp_acc_set_threshold(bsp_acc_t *ctx, float threshold_ms2)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (threshold_ms2 < 0.0f)
  {
    Serial.println("[ACC] WARNING: Threshold should be positive");
    return STATUS_ERROR;
  }

  ctx->config.accel_threshold_ms2 = threshold_ms2;
  Serial.printf("[ACC] Acceleration threshold set to %.2f m/s²\n", threshold_ms2);
  return STATUS_OK;
}

bool bsp_acc_is_calibrated(bsp_acc_t *ctx)
{
  if (ctx == nullptr)
  {
    return false;
  }

  return ctx->is_calibrated;
}

int8_t bsp_acc_scan_i2c(uint8_t sda_pin, uint8_t scl_pin)
{
  Serial.println("[ACC] Scanning I2C bus...");
  Wire.begin(sda_pin, scl_pin);

  int8_t found_addr = -1;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.printf("[ACC] Found device at 0x%02X\n", addr);
      if (addr == 0x6A || addr == 0x6B)
      {
        found_addr = addr;
        Serial.printf("[ACC] LSM6DS3 detected at 0x%02X\n", addr);
      }
    }
  }

  if (found_addr == -1)
  {
    Serial.println("[ACC] No LSM6DS3 found on I2C bus!");
    Serial.println("[ACC] Check wiring: SDA->GPIO4, SCL->GPIO5, VCC->3.3V, GND->GND");
  }

  return found_addr;
}

status_function_t bsp_acc_deinit(bsp_acc_t *ctx)
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
  ctx->is_calibrated  = false;
  ctx->callback       = nullptr;
  ctx->raw_callback   = nullptr;

  Serial.println("[ACC] Deinitialized");
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */

static float bsp_acc_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

/* End of file -------------------------------------------------------- */
