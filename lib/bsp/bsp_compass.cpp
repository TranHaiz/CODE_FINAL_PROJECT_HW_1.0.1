/**
 * @file       bsp_compass.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP implementation for HMC5883L Digital Compass Sensor
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_compass.h"

#include "device_pin_conf.h"
#include "os_lib.h"

#include <Arduino.h>
#include <math.h>

/* Private defines ---------------------------------------------------- */
#define COMPASS_OVERFLOW_VALUE (-4096)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static const char *s_direction_strings[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

/* Private function prototypes ---------------------------------------- */
static status_function_t bsp_compass_write_register(bsp_compass_t *ctx, uint8_t reg, uint8_t value);
static status_function_t bsp_compass_read_registers(bsp_compass_t *ctx, uint8_t reg, uint8_t *buffer, uint8_t len);
static bsp_compass_direction_t bsp_compass_deg_to_direction(float deg);
static const char             *bsp_compass_direction_to_str(bsp_compass_direction_t dir);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_compass_init(bsp_compass_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  // Set default configuration
  bsp_compass_config_t default_config = {
    .i2c_addr       = BSP_COMPASS_DEFAULT_I2C_ADDR,
    .sda_pin        = BSP_COMPASS_DEFAULT_SDA_PIN,
    .scl_pin        = BSP_COMPASS_DEFAULT_SCL_PIN,
    .i2c_clock      = BSP_COMPASS_DEFAULT_I2C_CLOCK,
    .data_rate      = BSP_COMPASS_RATE_15HZ,
    .gain           = BSP_COMPASS_GAIN_1_3GA,
    .mode           = BSP_COMPASS_MODE_CONTINUOUS,
    .averaging      = BSP_COMPASS_AVG_8,
    .ema_alpha      = BSP_COMPASS_DEFAULT_EMA_ALPHA,
    .update_rate_ms = BSP_COMPASS_DEFAULT_UPDATE_RATE_MS,
  };

  return bsp_compass_init_with_config(ctx, &default_config);
}

status_function_t bsp_compass_init_with_config(bsp_compass_t *ctx, const bsp_compass_config_t *config)
{
  if (ctx == nullptr || config == nullptr)
  {
    return STATUS_ERROR;
  }

  // Copy configuration
  ctx->config = *config;

  // Initialize data structures
  ctx->data.heading_deg   = 0.0f;
  ctx->data.heading_rad   = 0.0f;
  ctx->data.filtered_x    = 0.0f;
  ctx->data.filtered_y    = 0.0f;
  ctx->data.filtered_z    = 0.0f;
  ctx->data.direction     = BSP_COMPASS_DIR_N;
  ctx->data.direction_str = "N";
  ctx->data.timestamp_ms  = 0;
  ctx->data.is_valid      = false;
  ctx->data.is_overflow   = false;

  ctx->raw_data.raw_x = 0;
  ctx->raw_data.raw_y = 0;
  ctx->raw_data.raw_z = 0;

  // Initialize state
  ctx->callback       = nullptr;
  ctx->raw_callback   = nullptr;
  ctx->ema_x          = 0.0f;
  ctx->ema_y          = 0.0f;
  ctx->ema_z          = 0.0f;
  ctx->last_update_ms = 0;
  ctx->is_initialized = false;
  ctx->filter_ready   = false;

  Serial.printf("[COMPASS] Initialized with I2C addr=0x%02X, SDA=%d, SCL=%d\n", config->i2c_addr, config->sda_pin,
                config->scl_pin);

  return STATUS_OK;
}

status_function_t bsp_compass_begin(bsp_compass_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  // Initialize I2C
  Wire.begin(ctx->config.sda_pin, ctx->config.scl_pin, ctx->config.i2c_clock);
  delay(100);

  // Configure Register A: averaging + data rate
  uint8_t config_a = ctx->config.averaging | ctx->config.data_rate;
  if (bsp_compass_write_register(ctx, BSP_COMPASS_REG_CONFIG_A, config_a) != STATUS_OK)
  {
    Serial.println("[COMPASS] ERROR: Failed to write Config A");
    return STATUS_ERROR;
  }

  // Configure Register B: gain
  if (bsp_compass_write_register(ctx, BSP_COMPASS_REG_CONFIG_B, ctx->config.gain) != STATUS_OK)
  {
    Serial.println("[COMPASS] ERROR: Failed to write Config B");
    return STATUS_ERROR;
  }

  // Configure Mode Register
  if (bsp_compass_write_register(ctx, BSP_COMPASS_REG_MODE, ctx->config.mode) != STATUS_OK)
  {
    Serial.println("[COMPASS] ERROR: Failed to write Mode");
    return STATUS_ERROR;
  }

  ctx->is_initialized = true;
  ctx->last_update_ms = OS_GET_TICK();

  Serial.println("[COMPASS] Sensor started successfully");

  return STATUS_OK;
}

status_function_t bsp_compass_update(bsp_compass_t *ctx)
{
  if (ctx == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  // Read 6 bytes starting from data register
  uint8_t buffer[6];
  if (bsp_compass_read_registers(ctx, BSP_COMPASS_REG_DATA_X_MSB, buffer, 6) != STATUS_OK)
  {
    Serial.println("[COMPASS] ERROR: Failed to read data");
    bsp_compass_reset_i2c(ctx);
    return STATUS_ERROR;
  }

  // Parse raw data (HMC5883L order: X, Z, Y)
  int16_t raw_x = (int16_t) ((buffer[0] << 8) | buffer[1]);
  int16_t raw_z = (int16_t) ((buffer[2] << 8) | buffer[3]);
  int16_t raw_y = (int16_t) ((buffer[4] << 8) | buffer[5]);

  ctx->raw_data.raw_x = raw_x;
  ctx->raw_data.raw_y = raw_y;
  ctx->raw_data.raw_z = raw_z;

  // Check for overflow
  if (raw_x == COMPASS_OVERFLOW_VALUE || raw_y == COMPASS_OVERFLOW_VALUE || raw_z == COMPASS_OVERFLOW_VALUE)
  {
    ctx->data.is_overflow = true;
    ctx->data.is_valid    = false;
    Serial.println("[COMPASS] WARNING: Overflow detected - reduce gain");
    return STATUS_ERROR;
  }

  ctx->data.is_overflow = false;

  // Apply EMA filter
  if (!ctx->filter_ready)
  {
    ctx->ema_x        = (float) raw_x;
    ctx->ema_y        = (float) raw_y;
    ctx->ema_z        = (float) raw_z;
    ctx->filter_ready = true;
  }
  else
  {
    ctx->ema_x = ctx->config.ema_alpha * raw_x + (1.0f - ctx->config.ema_alpha) * ctx->ema_x;
    ctx->ema_y = ctx->config.ema_alpha * raw_y + (1.0f - ctx->config.ema_alpha) * ctx->ema_y;
    ctx->ema_z = ctx->config.ema_alpha * raw_z + (1.0f - ctx->config.ema_alpha) * ctx->ema_z;
  }

  // Update filtered values
  ctx->data.filtered_x = ctx->ema_x;
  ctx->data.filtered_y = ctx->ema_y;
  ctx->data.filtered_z = ctx->ema_z;

  // Calculate heading
  float heading_rad = atan2f(ctx->ema_y, ctx->ema_x);
  float heading_deg = heading_rad * 180.0f / M_PI;

  // Normalize to 0-360 degrees
  if (heading_deg < 0)
  {
    heading_deg += 360.0f;
  }

  ctx->data.heading_rad = heading_rad;
  ctx->data.heading_deg = heading_deg;

  // Get cardinal direction
  ctx->data.direction     = bsp_compass_deg_to_direction(heading_deg);
  ctx->data.direction_str = bsp_compass_direction_to_str(ctx->data.direction);

  // Update metadata
  ctx->data.timestamp_ms = OS_GET_TICK();
  ctx->data.is_valid     = true;

  // Call callbacks
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

status_function_t bsp_compass_read_raw(bsp_compass_t *ctx, bsp_compass_raw_data_t *data)
{
  if (ctx == nullptr || data == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  // Read 6 bytes starting from data register
  uint8_t buffer[6];
  if (bsp_compass_read_registers(ctx, BSP_COMPASS_REG_DATA_X_MSB, buffer, 6) != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  // Parse raw data (HMC5883L order: X, Z, Y)
  data->raw_x = (int16_t) ((buffer[0] << 8) | buffer[1]);
  data->raw_z = (int16_t) ((buffer[2] << 8) | buffer[3]);
  data->raw_y = (int16_t) ((buffer[4] << 8) | buffer[5]);

  return STATUS_OK;
}

status_function_t bsp_compass_get_data(bsp_compass_t *ctx, bsp_compass_data_t *data)
{
  if (ctx == nullptr || data == nullptr)
  {
    return STATUS_ERROR;
  }

  *data = ctx->data;
  return STATUS_OK;
}

status_function_t bsp_compass_get_heading_deg(bsp_compass_t *ctx, float *heading_deg)
{
  if (ctx == nullptr || heading_deg == nullptr)
  {
    return STATUS_ERROR;
  }

  *heading_deg = ctx->data.heading_deg;
  return STATUS_OK;
}

status_function_t bsp_compass_get_heading_rad(bsp_compass_t *ctx, float *heading_rad)
{
  if (ctx == nullptr || heading_rad == nullptr)
  {
    return STATUS_ERROR;
  }

  *heading_rad = ctx->data.heading_rad;
  return STATUS_OK;
}

status_function_t bsp_compass_get_direction(bsp_compass_t *ctx, bsp_compass_direction_t *direction)
{
  if (ctx == nullptr || direction == nullptr)
  {
    return STATUS_ERROR;
  }

  *direction = ctx->data.direction;
  return STATUS_OK;
}

status_function_t bsp_compass_get_direction_str(bsp_compass_t *ctx, const char **direction_str)
{
  if (ctx == nullptr || direction_str == nullptr)
  {
    return STATUS_ERROR;
  }

  *direction_str = ctx->data.direction_str;
  return STATUS_OK;
}

status_function_t bsp_compass_register_callback(bsp_compass_t *ctx, bsp_compass_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->callback = callback;
  return STATUS_OK;
}

status_function_t bsp_compass_register_raw_callback(bsp_compass_t *ctx, bsp_compass_raw_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->raw_callback = callback;
  return STATUS_OK;
}

status_function_t bsp_compass_set_filter_alpha(bsp_compass_t *ctx, float alpha)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (alpha < 0.0f || alpha > 1.0f)
  {
    Serial.println("[COMPASS] WARNING: Alpha should be between 0.0 and 1.0");
    return STATUS_ERROR;
  }

  ctx->config.ema_alpha = alpha;
  Serial.printf("[COMPASS] Filter alpha set to %.2f\n", alpha);
  return STATUS_OK;
}

status_function_t bsp_compass_set_gain(bsp_compass_t *ctx, bsp_compass_gain_t gain)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (bsp_compass_write_register(ctx, BSP_COMPASS_REG_CONFIG_B, gain) != STATUS_OK)
  {
    Serial.println("[COMPASS] ERROR: Failed to set gain");
    return STATUS_ERROR;
  }

  ctx->config.gain = gain;
  Serial.printf("[COMPASS] Gain set to 0x%02X\n", gain);
  return STATUS_OK;
}

status_function_t bsp_compass_set_rate(bsp_compass_t *ctx, bsp_compass_rate_t rate)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  uint8_t config_a = ctx->config.averaging | rate;
  if (bsp_compass_write_register(ctx, BSP_COMPASS_REG_CONFIG_A, config_a) != STATUS_OK)
  {
    Serial.println("[COMPASS] ERROR: Failed to set rate");
    return STATUS_ERROR;
  }

  ctx->config.data_rate = rate;
  Serial.printf("[COMPASS] Data rate set to 0x%02X\n", rate);
  return STATUS_OK;
}

status_function_t bsp_compass_reset_filter(bsp_compass_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->filter_ready = false;
  ctx->ema_x        = 0.0f;
  ctx->ema_y        = 0.0f;
  ctx->ema_z        = 0.0f;

  Serial.println("[COMPASS] Filter reset");
  return STATUS_OK;
}

status_function_t bsp_compass_reset_i2c(bsp_compass_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  Serial.println("[COMPASS] Resetting I2C bus...");

  // End current I2C session
  Wire.end();

  // Manually toggle SCL to release bus
  pinMode(ctx->config.scl_pin, OUTPUT);
  pinMode(ctx->config.sda_pin, OUTPUT);
  digitalWrite(ctx->config.sda_pin, HIGH);
  digitalWrite(ctx->config.scl_pin, HIGH);
  delay(10);

  // Clock out any stuck data
  for (int i = 0; i < 9; i++)
  {
    digitalWrite(ctx->config.scl_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(ctx->config.scl_pin, HIGH);
    delayMicroseconds(5);
  }

  // Generate STOP condition
  digitalWrite(ctx->config.sda_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(ctx->config.scl_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ctx->config.sda_pin, HIGH);
  delayMicroseconds(5);

  delay(100);

  // Reinitialize I2C
  Wire.begin(ctx->config.sda_pin, ctx->config.scl_pin, ctx->config.i2c_clock);
  delay(200);

  // Reconfigure sensor
  bsp_compass_write_register(ctx, BSP_COMPASS_REG_CONFIG_A, ctx->config.averaging | ctx->config.data_rate);
  bsp_compass_write_register(ctx, BSP_COMPASS_REG_CONFIG_B, ctx->config.gain);
  bsp_compass_write_register(ctx, BSP_COMPASS_REG_MODE, ctx->config.mode);

  Serial.println("[COMPASS] I2C bus reset complete");
  return STATUS_OK;
}

int8_t bsp_compass_scan_i2c(uint8_t sda_pin, uint8_t scl_pin)
{
  Serial.println("[COMPASS] Scanning I2C bus...");
  Wire.begin(sda_pin, scl_pin);

  int8_t found_addr = -1;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.printf("[COMPASS] Found device at 0x%02X\n", addr);
      if (addr == 0x1E)
      {
        found_addr = addr;
        Serial.printf("[COMPASS] HMC5883L detected at 0x%02X\n", addr);
      }
    }
  }

  if (found_addr == -1)
  {
    Serial.println("[COMPASS] No HMC5883L found on I2C bus!");
    Serial.println("[COMPASS] Check wiring: SDA->GPIO4, SCL->GPIO5, VCC->3.3V, GND->GND");
  }

  return found_addr;
}

bool bsp_compass_is_initialized(bsp_compass_t *ctx)
{
  if (ctx == nullptr)
  {
    return false;
  }

  return ctx->is_initialized;
}

status_function_t bsp_compass_deinit(bsp_compass_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->is_initialized = false;
  ctx->filter_ready   = false;
  ctx->callback       = nullptr;
  ctx->raw_callback   = nullptr;

  Serial.println("[COMPASS] Deinitialized");
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */

static status_function_t bsp_compass_write_register(bsp_compass_t *ctx, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ctx->config.i2c_addr);
  Wire.write(reg);
  Wire.write(value);

  if (Wire.endTransmission() != 0)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

static status_function_t bsp_compass_read_registers(bsp_compass_t *ctx, uint8_t reg, uint8_t *buffer, uint8_t len)
{
  // Set register pointer
  Wire.beginTransmission(ctx->config.i2c_addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0)
  {
    return STATUS_ERROR;
  }

  // Request data
  Wire.requestFrom(ctx->config.i2c_addr, len);
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

static bsp_compass_direction_t bsp_compass_deg_to_direction(float deg)
{
  if (deg < 22.5f || deg >= 337.5f)
    return BSP_COMPASS_DIR_N;
  if (deg < 67.5f)
    return BSP_COMPASS_DIR_NE;
  if (deg < 112.5f)
    return BSP_COMPASS_DIR_E;
  if (deg < 157.5f)
    return BSP_COMPASS_DIR_SE;
  if (deg < 202.5f)
    return BSP_COMPASS_DIR_S;
  if (deg < 247.5f)
    return BSP_COMPASS_DIR_SW;
  if (deg < 292.5f)
    return BSP_COMPASS_DIR_W;
  return BSP_COMPASS_DIR_NW;
}

static const char *bsp_compass_direction_to_str(bsp_compass_direction_t dir)
{
  if (dir >= 0 && dir < 8)
  {
    return s_direction_strings[dir];
  }
  return "?";
}

/* End of file -------------------------------------------------------- */
