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
    .i2c_addr       = BSP_ACC_I2C_ADDR,
    .sda_pin        = BSP_ACC_SDA_PIN,
    .scl_pin        = BSP_ACC_SCL_PIN,
    .update_rate_ms = BSP_ACC_DEFAULT_UPDATE_RATE_MS,
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
  Wire.begin(BSP_ACC_SDA_PIN, BSP_ACC_SCL_PIN);

  // Create LSM6DS3 sensor object
  ctx->sensor = new LSM6DS3(I2C_MODE, BSP_ACC_I2C_ADDR);

  if (ctx->sensor == nullptr)
  {
    Serial.println("[ACC] ERROR: Failed to create sensor object");
    return STATUS_ERROR;
  }

  // Initialize data structures
  ctx->data.accel_magnitude = 0.0f;
  ctx->data.timestamp_ms    = 0;
  ctx->data.is_valid        = false;

  ctx->raw_data.accel_x = 0.0f;
  ctx->raw_data.accel_y = 0.0f;
  ctx->raw_data.accel_z = 0.0f;
  ctx->raw_data.gyro_x  = 0.0f;
  ctx->raw_data.gyro_y  = 0.0f;
  ctx->raw_data.gyro_z  = 0.0f;
  ctx->raw_data.temp_c  = 0.0f;

  // Initialize state
  ctx->callback       = nullptr;
  ctx->raw_callback   = nullptr;
  ctx->last_update_us = 0;
  ctx->last_update_ms = 0;
  ctx->is_initialized = false;

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

status_function_t bsp_acc_update(bsp_acc_t *ctx)
{
  if (ctx == nullptr || ctx->sensor == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  // Update timestamps
  ctx->last_update_us = micros();
  ctx->last_update_ms = OS_GET_TICK();

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

  // Update timestamp and validity
  ctx->data.timestamp_ms = ctx->last_update_ms;
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

status_function_t bsp_acc_get_magnitude(bsp_acc_t *ctx, float *mag_g)
{
  if (ctx == nullptr || mag_g == nullptr)
  {
    return STATUS_ERROR;
  }

  *mag_g = ctx->data.accel_magnitude;
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

bool bsp_acc_is_initialized(bsp_acc_t *ctx)
{
  if (ctx == nullptr)
  {
    return false;
  }

  return ctx->is_initialized;
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
