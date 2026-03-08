/**
 * @file       bsp_acc.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      BSP implementation for LSM6DS3 Accelerometer/Gyroscope Sensor (Singleton)
 *
 * @note       This module provides low-level hardware access only.
 *             Processing should be done at system layer.
 *             Only supports single instance.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_acc.h"

#include "device_pin_conf.h"
#include "os_lib.h"

#include <Arduino.h>
#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <math.h>

/* Private defines ---------------------------------------------------- */
#define BSP_ACC_I2C_ADDR     (0x6B)
#define BSP_ACC_I2C_ADDR_ALT (0x6A)

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Internal accelerometer context (singleton)
 */
typedef struct
{
  LSM6DS3           *sensor;
  bsp_acc_data_t     data;
  bsp_acc_raw_data_t raw_data;
  size_t             last_update_ms;
  bool               is_initialized;
} bsp_acc_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_acc_ctx_t s_acc_ctx = {
  .sensor         = nullptr,
  .data           = { 0 },
  .raw_data       = { 0 },
  .last_update_ms = 0,
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
static float bsp_acc_calculate_magnitude(float x, float y, float z);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_acc_init(void)
{
  if (s_acc_ctx.is_initialized)
  {
    return STATUS_OK;
  }

  // Initialize I2C
  Wire.begin(ACC_I2C_SDA_PIN, ACC_I2C_SCL_PIN);
  delay(100);

  // Create LSM6DS3 sensor object with primary address
  s_acc_ctx.sensor = new LSM6DS3(I2C_MODE, BSP_ACC_I2C_ADDR);
  if (s_acc_ctx.sensor == nullptr)
  {
    Serial.println("[ACC] ERROR: Failed to create sensor object");
    return STATUS_ERROR;
  }

  // Try to start sensor
  uint8_t result = s_acc_ctx.sensor->begin();

  // If failed, try alternate I2C address
  if (result != 0)
  {
    Serial.printf("[ACC] Failed with 0x%02X, trying 0x%02X...\n", BSP_ACC_I2C_ADDR, BSP_ACC_I2C_ADDR_ALT);

    delete s_acc_ctx.sensor;
    s_acc_ctx.sensor = new LSM6DS3(I2C_MODE, BSP_ACC_I2C_ADDR_ALT);

    if (s_acc_ctx.sensor == nullptr || s_acc_ctx.sensor->begin() != 0)
    {
      Serial.println("[ACC] ERROR: Failed to start sensor on both addresses");
      return STATUS_ERROR;
    }

    Serial.printf("[ACC] Connected on alternate address 0x%02X\n", BSP_ACC_I2C_ADDR_ALT);
  }

  // Initialize data structures
  s_acc_ctx.data.accel_magnitude = 0.0f;
  s_acc_ctx.data.timestamp_ms    = 0;
  s_acc_ctx.data.is_valid        = false;

  memset(&s_acc_ctx.raw_data, 0, sizeof(bsp_acc_raw_data_t));

  s_acc_ctx.last_update_ms = OS_GET_TICK();
  s_acc_ctx.is_initialized = true;

  Serial.println("[ACC] Sensor initialized successfully");

  return STATUS_OK;
}

status_function_t bsp_acc_read_raw(bsp_acc_raw_data_t *data)
{
  if (data == nullptr || !s_acc_ctx.is_initialized || s_acc_ctx.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Read accelerometer data
  data->accel_x = s_acc_ctx.sensor->readFloatAccelX();
  data->accel_y = s_acc_ctx.sensor->readFloatAccelY();
  data->accel_z = s_acc_ctx.sensor->readFloatAccelZ();

  // Read gyroscope data
  data->gyro_x = s_acc_ctx.sensor->readFloatGyroX();
  data->gyro_y = s_acc_ctx.sensor->readFloatGyroY();
  data->gyro_z = s_acc_ctx.sensor->readFloatGyroZ();

  // Read temperature
  data->temp_c = s_acc_ctx.sensor->readTempC();

  // Update internal cache
  s_acc_ctx.raw_data       = *data;
  s_acc_ctx.last_update_ms = OS_GET_TICK();

  // Calculate magnitude and update processed data
  s_acc_ctx.data.accel_magnitude = bsp_acc_calculate_magnitude(data->accel_x, data->accel_y, data->accel_z);
  s_acc_ctx.data.timestamp_ms    = s_acc_ctx.last_update_ms;
  s_acc_ctx.data.is_valid        = true;

  return STATUS_OK;
}

status_function_t bsp_acc_get_data(bsp_acc_data_t *data)
{
  if (data == nullptr || !s_acc_ctx.is_initialized)
  {
    return STATUS_ERROR;
  }

  *data = s_acc_ctx.data;
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */

static float bsp_acc_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

/* End of file -------------------------------------------------------- */
