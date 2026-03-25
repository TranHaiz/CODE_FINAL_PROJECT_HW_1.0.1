/**
 * @file       bsp_compass.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-03-02
 * @author     Hai Tran
 *
 * @brief      BSP implementation for HMC5883L Digital Compass Sensor (Singleton)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_compass.h"

#include "device_config.h"
#include "log_service.h"

#include <Arduino.h>
#include <Wire.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_compass, LOG_LEVEL_ERROR);
#define BSP_COMPASS_REG_CONFIG_A   (0x00)
#define BSP_COMPASS_REG_CONFIG_B   (0x01)
#define BSP_COMPASS_REG_MODE       (0x02)
#define BSP_COMPASS_REG_DATA_X_MSB (0x03)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  bsp_compass_config_t config;
  bool                 is_initialized;
} bsp_compass_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_compass_ctx_t s_compass_ctx = {
  .config = {
    .data_rate = BSP_COMPASS_RATE_15HZ,
    .gain      = BSP_COMPASS_GAIN_1_3GA,
    .mode      = BSP_COMPASS_MODE_CONTINUOUS,
    .averaging = BSP_COMPASS_AVG_8,
  },
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
static status_function_t bsp_compass_write_register(uint8_t reg, uint8_t value);
static status_function_t bsp_compass_read_registers(uint8_t reg, uint8_t *buffer, uint8_t len);
static status_function_t bsp_compass_apply_config(void);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_compass_init(void)
{
  if (s_compass_ctx.is_initialized)
  {
    return STATUS_OK;
  }

  // Set default configuration
  s_compass_ctx.config.data_rate = BSP_COMPASS_RATE_15HZ;
  s_compass_ctx.config.gain      = BSP_COMPASS_GAIN_1_3GA;
  s_compass_ctx.config.mode      = BSP_COMPASS_MODE_CONTINUOUS;
  s_compass_ctx.config.averaging = BSP_COMPASS_AVG_8;

  // Init I2C
  Wire.begin(COMPASS_I2C_SDA_PIN, COMPASS_I2C_SCL_PIN, COMPASS_I2C_CLOCK);
  delay(100);

  // Configure sensor
  if (bsp_compass_apply_config() != STATUS_OK)
  {
    LOG_ERR("Failed to init compass sensor");
    return STATUS_ERROR;
  }

  s_compass_ctx.is_initialized = true;

  return STATUS_OK;
}

status_function_t bsp_compass_config(const bsp_compass_config_t *config)
{
  if ((config == nullptr) || !s_compass_ctx.is_initialized)
  {
    return STATUS_ERROR;
  }

  // Update configuration
  s_compass_ctx.config = *config;

  // Apply new configuration
  return bsp_compass_apply_config();
}

status_function_t bsp_compass_read_raw(bsp_compass_raw_data_t *data)
{
  if (data == nullptr || !s_compass_ctx.is_initialized)
  {
    return STATUS_ERROR;
  }

  // Read 6 bytes starting from data register
  uint8_t buffer[6];
  if (bsp_compass_read_registers(BSP_COMPASS_REG_DATA_X_MSB, buffer, 6) != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  // Parse raw data (HMC5883L order: X, Z, Y)
  data->raw_x = (int16_t) ((buffer[0] << 8) | buffer[1]);
  data->raw_z = (int16_t) ((buffer[2] << 8) | buffer[3]);
  data->raw_y = (int16_t) ((buffer[4] << 8) | buffer[5]);

  // Check for overflow
  if (data->raw_x == BSP_COMPASS_OVERFLOW_VALUE || data->raw_y == BSP_COMPASS_OVERFLOW_VALUE
      || data->raw_z == BSP_COMPASS_OVERFLOW_VALUE)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
static status_function_t bsp_compass_apply_config(void)
{
  // Configure Register A: averaging + data rate
  uint8_t config_a = s_compass_ctx.config.averaging | s_compass_ctx.config.data_rate;
  if (bsp_compass_write_register(BSP_COMPASS_REG_CONFIG_A, config_a) != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  // Configure Register B: gain
  if (bsp_compass_write_register(BSP_COMPASS_REG_CONFIG_B, s_compass_ctx.config.gain) != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  // Configure Mode Register
  if (bsp_compass_write_register(BSP_COMPASS_REG_MODE, s_compass_ctx.config.mode) != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

static status_function_t bsp_compass_write_register(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(COMPASS_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);

  if (Wire.endTransmission() != 0)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

static status_function_t bsp_compass_read_registers(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  // Set register pointer
  Wire.beginTransmission(COMPASS_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    return STATUS_ERROR;
  }

  // Request data
  Wire.requestFrom((uint8_t) COMPASS_I2C_ADDR, len);
  if (Wire.available() < len)
  {
    return STATUS_ERROR;
  }

  for (uint8_t i = 0; i < len; i++)
  {
    buffer[i] = Wire.read();
  }

  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
