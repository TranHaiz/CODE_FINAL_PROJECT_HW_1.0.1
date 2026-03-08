/**
 * @file       sys_input.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      System Input Layer - GPS/INS Sensor Fusion Implementation
 *
 * @details    Implements GPS/INS integration using complementary filter:
 *
 *             Velocity Fusion Algorithm:
 *             1. Read IMU acceleration data at high frequency (50Hz)
 *             2. Apply low-pass filter to remove noise
 *             3. Remove gravity offset (from calibration)
 *             4. Integrate acceleration to get INS velocity
 *             5. When GPS is valid, apply complementary filter:
 *                v_fused = k * v_gps + (1-k) * v_ins
 *             6. When GPS invalid, use INS velocity with decay
 *
 *             Reference: GPS/INS Integration methodology
 */

/* Includes ----------------------------------------------------------- */
#include "sys_input.h"

#include "os_lib.h"

#include <Arduino.h>
#include <math.h>

/* Private defines ---------------------------------------------------- */
#define SYS_INPUT_CALIBRATION_SAMPLES (200)
#define SYS_INPUT_CALIBRATION_DELAY   (5)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static float sys_input_calculate_magnitude(float x, float y, float z);
static void  sys_input_update_ins_velocity(sys_input_t *ctx, float dt);
static void  sys_input_update_gps_data(sys_input_t *ctx);
static void  sys_input_fuse_velocity(sys_input_t *ctx);
static void  sys_input_update_heading(sys_input_t *ctx);

/* Function definitions ----------------------------------------------- */

status_function_t sys_input_init(sys_input_t *ctx, bsp_acc_t *acc_ctx, bsp_compass_t *compass_ctx)
{
  if (ctx == nullptr || acc_ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  // Set default configuration
  sys_input_config_t default_config = {
    .fusion_mode         = SYS_INPUT_MODE_GPS_INS_FUSION,
    .filter_alpha        = SYS_INPUT_DEFAULT_FILTER_ALPHA,
    .comp_filter_k       = SYS_INPUT_DEFAULT_COMP_FILTER_K,
    .accel_threshold_ms2 = SYS_INPUT_DEFAULT_ACCEL_THRESHOLD,
    .friction_factor     = SYS_INPUT_DEFAULT_FRICTION_FACTOR,
    .update_rate_ms      = SYS_INPUT_DEFAULT_UPDATE_RATE_MS,
    .gps_timeout_ms      = SYS_INPUT_GPS_VALID_TIMEOUT_MS,
  };

  return sys_input_init_with_config(ctx, acc_ctx, compass_ctx, &default_config);
}

status_function_t sys_input_init_with_config(sys_input_t              *ctx,
                                             bsp_acc_t                *acc_ctx,
                                             bsp_compass_t            *compass_ctx,
                                             const sys_input_config_t *config)
{
  if (ctx == nullptr || acc_ctx == nullptr || config == nullptr)
  {
    return STATUS_ERROR;
  }

  // Copy configuration
  ctx->config = *config;

  // Store sensor contexts
  ctx->acc_ctx     = acc_ctx;
  ctx->compass_ctx = compass_ctx;

  // Initialize velocity data
  ctx->data.velocity.velocity_ms    = 0.0f;
  ctx->data.velocity.velocity_kmh   = 0.0f;
  ctx->data.velocity.velocity_gps   = 0.0f;
  ctx->data.velocity.velocity_ins   = 0.0f;
  ctx->data.velocity.heading_deg    = 0.0f;
  ctx->data.velocity.heading_rad    = 0.0f;
  ctx->data.velocity.accel_ms2      = 0.0f;
  ctx->data.velocity.accel_filtered = 0.0f;
  ctx->data.velocity.timestamp_ms   = 0;
  ctx->data.velocity.is_valid       = false;

  // Initialize position data
  ctx->data.position.latitude     = 0.0;
  ctx->data.position.longitude    = 0.0;
  ctx->data.position.altitude     = 0.0;
  ctx->data.position.course_deg   = 0.0;
  ctx->data.position.satellites   = 0;
  ctx->data.position.hdop         = 99.9;
  ctx->data.position.timestamp_ms = 0;
  ctx->data.position.is_valid     = false;

  // Initialize status
  ctx->data.status.gps_valid      = false;
  ctx->data.status.gps_recent     = false;
  ctx->data.status.imu_calibrated = false;
  ctx->data.status.compass_valid  = false;

  // Initialize mode
  ctx->data.mode = config->fusion_mode;

  // Initialize state
  ctx->vel_callback     = nullptr;
  ctx->pos_callback     = nullptr;
  ctx->offset_magnitude = 0.0f;
  ctx->accel_filtered   = 0.0f;
  ctx->last_update_us   = 0;
  ctx->last_update_ms   = 0;
  ctx->last_gps_ms      = 0;
  ctx->is_initialized   = true;
  ctx->is_calibrated    = false;

  Serial.println("[SYS_INPUT] Initialized GPS/INS fusion system");
  Serial.printf("[SYS_INPUT] Mode: %d, Filter_K: %.2f, Alpha: %.2f\n", config->fusion_mode, config->comp_filter_k,
                config->filter_alpha);

  return STATUS_OK;
}

status_function_t sys_input_calibrate(sys_input_t *ctx)
{
  if (ctx == nullptr || ctx->acc_ctx == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  Serial.println("[SYS_INPUT] Starting IMU calibration... Keep sensor stationary!");

  bsp_acc_raw_data_t raw_data;
  float              sum_magnitude = 0.0f;

  for (uint16_t i = 0; i < SYS_INPUT_CALIBRATION_SAMPLES; i++)
  {
    if (bsp_acc_read_raw(ctx->acc_ctx, &raw_data) == STATUS_OK)
    {
      sum_magnitude += sys_input_calculate_magnitude(raw_data.accel_x, raw_data.accel_y, raw_data.accel_z);
    }
    delay(SYS_INPUT_CALIBRATION_DELAY);
  }

  ctx->offset_magnitude           = sum_magnitude / (float) SYS_INPUT_CALIBRATION_SAMPLES;
  ctx->is_calibrated              = true;
  ctx->data.status.imu_calibrated = true;
  ctx->last_update_us             = micros();
  ctx->last_update_ms             = OS_GET_TICK();

  Serial.printf("[SYS_INPUT] Calibration complete. Gravity offset: %.4f g\n", ctx->offset_magnitude);

  return STATUS_OK;
}

status_function_t sys_input_update(sys_input_t *ctx)
{
  if (ctx == nullptr || ctx->acc_ctx == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  // Calculate time delta
  size_t current_time_us = micros();
  float  dt              = (current_time_us - ctx->last_update_us) / 1000000.0f;
  ctx->last_update_us    = current_time_us;
  ctx->last_update_ms    = OS_GET_TICK();

  // Limit dt to avoid large jumps
  if (dt > 0.1f)
  {
    dt = 0.1f;
  }

  // Read raw IMU data
  bsp_acc_read_raw(ctx->acc_ctx, &ctx->data.imu_raw);

  // Update GPS data
  sys_input_update_gps_data(ctx);

  // Update heading from compass if available
  sys_input_update_heading(ctx);

  // Update INS velocity if calibrated
  if (ctx->is_calibrated)
  {
    sys_input_update_ins_velocity(ctx, dt);
  }

  // Perform sensor fusion based on mode
  sys_input_fuse_velocity(ctx);

  // Update timestamp and validity
  ctx->data.velocity.timestamp_ms = ctx->last_update_ms;
  ctx->data.velocity.is_valid     = true;

  // Call velocity callback if registered
  if (ctx->vel_callback != nullptr)
  {
    ctx->vel_callback(&ctx->data.velocity);
  }

  return STATUS_OK;
}

status_function_t sys_input_get_velocity(sys_input_t *ctx, sys_input_velocity_data_t *data)
{
  if (ctx == nullptr || data == nullptr)
  {
    return STATUS_ERROR;
  }

  *data = ctx->data.velocity;
  return STATUS_OK;
}

status_function_t sys_input_get_position(sys_input_t *ctx, sys_input_position_data_t *data)
{
  if (ctx == nullptr || data == nullptr)
  {
    return STATUS_ERROR;
  }

  *data = ctx->data.position;
  return STATUS_OK;
}

status_function_t sys_input_get_data(sys_input_t *ctx, sys_input_data_t *data)
{
  if (ctx == nullptr || data == nullptr)
  {
    return STATUS_ERROR;
  }

  *data = ctx->data;
  return STATUS_OK;
}

status_function_t sys_input_get_velocity_ms(sys_input_t *ctx, float *velocity_ms)
{
  if (ctx == nullptr || velocity_ms == nullptr)
  {
    return STATUS_ERROR;
  }

  *velocity_ms = ctx->data.velocity.velocity_ms;
  return STATUS_OK;
}

status_function_t sys_input_get_velocity_kmh(sys_input_t *ctx, float *velocity_kmh)
{
  if (ctx == nullptr || velocity_kmh == nullptr)
  {
    return STATUS_ERROR;
  }

  *velocity_kmh = ctx->data.velocity.velocity_kmh;
  return STATUS_OK;
}

status_function_t sys_input_get_heading(sys_input_t *ctx, float *heading_deg)
{
  if (ctx == nullptr || heading_deg == nullptr)
  {
    return STATUS_ERROR;
  }

  *heading_deg = ctx->data.velocity.heading_deg;
  return STATUS_OK;
}

status_function_t sys_input_get_accel(sys_input_t *ctx, float *accel_ms2)
{
  if (ctx == nullptr || accel_ms2 == nullptr)
  {
    return STATUS_ERROR;
  }

  *accel_ms2 = ctx->data.velocity.accel_ms2;
  return STATUS_OK;
}

status_function_t sys_input_get_status(sys_input_t *ctx, sys_input_sensor_status_t *status)
{
  if (ctx == nullptr || status == nullptr)
  {
    return STATUS_ERROR;
  }

  *status = ctx->data.status;
  return STATUS_OK;
}

status_function_t sys_input_reset_velocity(sys_input_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->data.velocity.velocity_ms  = 0.0f;
  ctx->data.velocity.velocity_kmh = 0.0f;
  ctx->data.velocity.velocity_ins = 0.0f;
  ctx->accel_filtered             = 0.0f;

  Serial.println("[SYS_INPUT] Velocity reset to zero");
  return STATUS_OK;
}

status_function_t sys_input_set_mode(sys_input_t *ctx, sys_input_fusion_mode_t mode)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->config.fusion_mode = mode;
  ctx->data.mode          = mode;

  Serial.printf("[SYS_INPUT] Fusion mode set to %d\n", mode);
  return STATUS_OK;
}

status_function_t sys_input_set_comp_filter_k(sys_input_t *ctx, float k)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (k < 0.0f || k > 1.0f)
  {
    Serial.println("[SYS_INPUT] WARNING: Filter K should be between 0.0 and 1.0");
    return STATUS_ERROR;
  }

  ctx->config.comp_filter_k = k;
  Serial.printf("[SYS_INPUT] Complementary filter K set to %.2f\n", k);
  return STATUS_OK;
}

status_function_t sys_input_set_filter_alpha(sys_input_t *ctx, float alpha)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (alpha < 0.0f || alpha > 1.0f)
  {
    Serial.println("[SYS_INPUT] WARNING: Alpha should be between 0.0 and 1.0");
    return STATUS_ERROR;
  }

  ctx->config.filter_alpha = alpha;
  Serial.printf("[SYS_INPUT] Filter alpha set to %.2f\n", alpha);
  return STATUS_OK;
}

status_function_t sys_input_set_threshold(sys_input_t *ctx, float threshold_ms2)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (threshold_ms2 < 0.0f)
  {
    Serial.println("[SYS_INPUT] WARNING: Threshold should be positive");
    return STATUS_ERROR;
  }

  ctx->config.accel_threshold_ms2 = threshold_ms2;
  Serial.printf("[SYS_INPUT] Acceleration threshold set to %.2f m/s²\n", threshold_ms2);
  return STATUS_OK;
}

status_function_t sys_input_register_velocity_callback(sys_input_t *ctx, sys_input_velocity_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->vel_callback = callback;
  return STATUS_OK;
}

status_function_t sys_input_register_position_callback(sys_input_t *ctx, sys_input_position_callback_t callback)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->pos_callback = callback;
  return STATUS_OK;
}

bool sys_input_is_calibrated(sys_input_t *ctx)
{
  if (ctx == nullptr)
  {
    return false;
  }

  return ctx->is_calibrated;
}

bool sys_input_is_gps_available(sys_input_t *ctx)
{
  if (ctx == nullptr)
  {
    return false;
  }

  return ctx->data.status.gps_valid && ctx->data.status.gps_recent;
}

status_function_t sys_input_deinit(sys_input_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  ctx->is_initialized = false;
  ctx->is_calibrated  = false;
  ctx->vel_callback   = nullptr;
  ctx->pos_callback   = nullptr;
  ctx->acc_ctx        = nullptr;
  ctx->compass_ctx    = nullptr;

  Serial.println("[SYS_INPUT] Deinitialized");
  return STATUS_OK;
}

/* Private function definitions --------------------------------------- */

static float sys_input_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

/**
 * @brief Update INS velocity by integrating acceleration
 */
static void sys_input_update_ins_velocity(sys_input_t *ctx, float dt)
{
  // Calculate current acceleration magnitude
  float current_magnitude =
    sys_input_calculate_magnitude(ctx->data.imu_raw.accel_x, ctx->data.imu_raw.accel_y, ctx->data.imu_raw.accel_z);

  // Remove gravity offset
  float accel_clear = current_magnitude - ctx->offset_magnitude;

  // Apply low-pass filter
  ctx->accel_filtered =
    ctx->config.filter_alpha * accel_clear + (1.0f - ctx->config.filter_alpha) * ctx->accel_filtered;

  // Convert to m/s²
  ctx->data.velocity.accel_filtered = ctx->accel_filtered;
  ctx->data.velocity.accel_ms2      = ctx->accel_filtered * SYS_INPUT_GRAVITY_MS2;

  // Integrate to calculate INS velocity
  if (fabsf(ctx->data.velocity.accel_ms2) > ctx->config.accel_threshold_ms2)
  {
    ctx->data.velocity.velocity_ins += ctx->data.velocity.accel_ms2 * dt;
  }
  else
  {
    // Apply friction when acceleration is below threshold
    ctx->data.velocity.velocity_ins *= ctx->config.friction_factor;
  }

  // Ensure INS velocity is not negative
  if (ctx->data.velocity.velocity_ins < 0.0f)
  {
    ctx->data.velocity.velocity_ins = 0.0f;
  }
}

/**
 * @brief Update GPS data and status
 */
static void sys_input_update_gps_data(sys_input_t *ctx)
{
  bsp_gps_data_t gps_data;

  if (bsp_gps_get_data(&gps_data) == STATUS_OK)
  {
    // Store GPS data
    ctx->data.gps = gps_data;

    // Update position
    if (gps_data.location_valid)
    {
      ctx->data.position.latitude     = gps_data.latitude;
      ctx->data.position.longitude    = gps_data.longitude;
      ctx->data.position.altitude     = gps_data.altitude;
      ctx->data.position.course_deg   = gps_data.course;
      ctx->data.position.satellites   = gps_data.satellites;
      ctx->data.position.hdop         = gps_data.hdop;
      ctx->data.position.timestamp_ms = ctx->last_update_ms;
      ctx->data.position.is_valid     = true;

      // Update GPS velocity (convert km/h to m/s)
      ctx->data.velocity.velocity_gps = (float) (gps_data.speed_kmph / 3.6);

      // Update GPS status
      ctx->data.status.gps_valid = true;
      ctx->last_gps_ms           = ctx->last_update_ms;

      // Call position callback if registered
      if (ctx->pos_callback != nullptr)
      {
        ctx->pos_callback(&ctx->data.position);
      }
    }
    else
    {
      ctx->data.status.gps_valid = false;
    }
  }
  else
  {
    ctx->data.status.gps_valid = false;
  }

  // Check if GPS data is recent
  size_t gps_age              = ctx->last_update_ms - ctx->last_gps_ms;
  ctx->data.status.gps_recent = (gps_age < ctx->config.gps_timeout_ms) && ctx->data.status.gps_valid;
}

/**
 * @brief Update heading from compass or GPS
 */
static void sys_input_update_heading(sys_input_t *ctx)
{
  // Try compass first
  if (ctx->compass_ctx != nullptr)
  {
    bsp_compass_data_t compass_data;
    if (bsp_compass_get_data(ctx->compass_ctx, &compass_data) == STATUS_OK && compass_data.is_valid)
    {
      ctx->data.compass              = compass_data;
      ctx->data.velocity.heading_deg = compass_data.heading_deg;
      ctx->data.velocity.heading_rad = compass_data.heading_rad;
      ctx->data.status.compass_valid = true;
      return;
    }
  }

  // Fallback to GPS course if compass not available
  if (ctx->data.status.gps_valid && ctx->data.velocity.velocity_gps > 1.0f)
  {
    ctx->data.velocity.heading_deg = (float) ctx->data.gps.course;
    ctx->data.velocity.heading_rad = ctx->data.velocity.heading_deg * M_PI / 180.0f;
  }

  ctx->data.status.compass_valid = false;
}

/**
 * @brief Perform sensor fusion based on selected mode
 *
 * Complementary Filter:
 *   v_fused = k * v_gps + (1-k) * v_ins
 *
 * Where k is the GPS weight (higher = more trust in GPS)
 * Typical k = 0.98 (GPS dominates when available)
 */
static void sys_input_fuse_velocity(sys_input_t *ctx)
{
  float fused_velocity = 0.0f;

  switch (ctx->config.fusion_mode)
  {
  case SYS_INPUT_MODE_GPS_ONLY:
    // Use GPS velocity only
    if (ctx->data.status.gps_valid && ctx->data.status.gps_recent)
    {
      fused_velocity = ctx->data.velocity.velocity_gps;
    }
    else
    {
      fused_velocity = 0.0f;
    }
    break;

  case SYS_INPUT_MODE_INS_ONLY:
    // Use INS velocity only
    fused_velocity = ctx->data.velocity.velocity_ins;
    break;

  case SYS_INPUT_MODE_GPS_INS_FUSION:
    // Complementary filter fusion
    if (ctx->data.status.gps_valid && ctx->data.status.gps_recent)
    {
      // GPS available: apply complementary filter
      float k        = ctx->config.comp_filter_k;
      fused_velocity = k * ctx->data.velocity.velocity_gps + (1.0f - k) * ctx->data.velocity.velocity_ins;

      // Correct INS drift: slowly pull INS towards GPS
      ctx->data.velocity.velocity_ins = fused_velocity;
    }
    else if (ctx->is_calibrated)
    {
      // GPS not available: use INS with decay
      fused_velocity = ctx->data.velocity.velocity_ins;
      ctx->data.velocity.velocity_ins *= SYS_INPUT_INS_DECAY_FACTOR;
    }
    break;

  case SYS_INPUT_MODE_AUTO:
    // Auto mode: prefer GPS when available, fallback to INS
    if (ctx->data.status.gps_valid && ctx->data.status.gps_recent)
    {
      // Good GPS: use GPS with light INS smoothing
      float k        = 0.9f;
      fused_velocity = k * ctx->data.velocity.velocity_gps + (1.0f - k) * ctx->data.velocity.velocity_ins;
      ctx->data.velocity.velocity_ins = fused_velocity;
    }
    else if (ctx->is_calibrated)
    {
      // No GPS: use INS
      fused_velocity = ctx->data.velocity.velocity_ins;
      ctx->data.velocity.velocity_ins *= SYS_INPUT_INS_DECAY_FACTOR;
    }
    break;

  default: fused_velocity = 0.0f; break;
  }

  // Ensure velocity is not negative
  if (fused_velocity < 0.0f)
  {
    fused_velocity = 0.0f;
  }

  // Update fused velocity
  ctx->data.velocity.velocity_ms  = fused_velocity;
  ctx->data.velocity.velocity_kmh = fused_velocity * 3.6f;
}

/* End of file -------------------------------------------------------- */
