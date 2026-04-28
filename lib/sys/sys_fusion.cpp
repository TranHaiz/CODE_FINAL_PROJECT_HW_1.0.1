/**
 * @file       sys_fusion.cpp
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-25
 * @author     Hai Tran
 *
 * @brief      Sensor fusion implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_fusion.h"

#include "bsp_acc.h"
#include "bsp_compass.h"
#include "bsp_gps.h"
#include "log_service.h"
#include "os_lib.h"

#include <SimpleKalmanFilter.h>  // platformio.ini: lib_deps = denyssene/SimpleKalmanFilter
#include <math.h>
#include <stdlib.h>  // calloc

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_fusion, LOG_LEVEL_INFO)

#define DEMO_VEHICLE                (true)
#define DEMO_WALKING                (false)

// Accelerometer parameters
#define ACC_KF_E_MEA                (0.05f)
#define ACC_KF_E_EST                (0.05f)
#define ACC_KF_Q                    (0.1f)
#define ACC_THRESHOLD_MS2           (0.05f)  // Dead-band to gate INS integration (m/s²)
#define ACC_OFFSET_MAGNITUDE_SAMPLE (200)

// Attitude complementary filter (gyro + accelerometer)
#define ATTITUDE_GYRO_WEIGHT        (0.95f)
#define GYRO_BIAS_CALIB_SAMPLES     (200)
#define GYRO_BIAS_ALPHA             (0.01f)
#define ACC_FORWARD_MAX_MS2         (6.0f)

// Gyro Kalman filter — low Q = trust smoothed estimate; raise Q if attitude lags fast turns
#define GYRO_KF_E_MEA               (0.5f)  // measurement noise (dps units)
#define GYRO_KF_E_EST               (0.5f)  // initial estimate error
#define GYRO_KF_Q                   (0.5f)  // process noise: lower = smoother, higher = more responsive

// Velocity complementary filter crossover frequency (rad/s)  [Zhao 2020]
#define CF_WC                       (1.0f)
#define INS_BLEND_ACTIVE            (0.3f)  // GPS ACTIVE + reliable
#define INS_BLEND_FADING            (0.1f)  // GPS FADING — cautious
#define INS_BLEND_INVALID           (0.0f)  // GPS lost — pure CF

#if (DEMO_VEHICLE)
#define ZUPT_ACC_THRESHOLD          (0.03f)
#define ZUPT_TIME_THRESHOLD_MS      (1000)
#define INS_DECAY_NORMAL            (0.9990f)
#define INS_DECAY_STOPPING          (0.94f)
#define INS_DECAY_GPS_LOST          (0.97f)
#define GPS_SPEED_MIN_MS            (0.6f)
#define GPS_ANCHOR_RATE             (0.7f)
#define GPS_RELIABILITY_THRESHOLD_M (20.0f)

#elif (DEMO_WALKING)
#define ZUPT_ACC_THRESHOLD          (0.015f)
#define ZUPT_TIME_THRESHOLD_MS      (1500)
#define INS_DECAY_NORMAL            (0.9995f)  // Slow decay while walking
#define INS_DECAY_STOPPING          (0.92f)    // Fast decay ~0.5s to zero
#define INS_DECAY_GPS_LOST          (0.96f)    // Medium decay when GPS fading out
#define GPS_SPEED_MIN_MS            (0.4f)     // 1.4 km/h
#define GPS_ANCHOR_RATE             (0.7f)     // Stronger anchor = more responsive
#define GPS_RELIABILITY_THRESHOLD_M (10.0f)    // Max |d_INS - d_GPS| before GPS rejected

#else
#error "Must define either DEMO_VEHICLE or DEMO_WALKING"
#endif

#define GPS_HDOP_MAX                  (3.0f)
#define GPS_SATELLITES_MIN            (4)
#define GPS_EMA_ALPHA                 (0.6f)
#define GPS_MAX_STEP_M                (50.0f)
#define GPS_VALID_TIMEOUT_MS          (2000)
#define GPS_FADE_TIMEOUT_MS           (1000)

// Compass Kalman filter — low Q suppresses magnetic noise; raise Q if heading reacts too slowly
#define COMPASS_KF_E_MEA              (2.0f)   // measurement noise (raw counts)
#define COMPASS_KF_E_EST              (2.0f)   // initial estimate error
#define COMPASS_KF_Q                  (0.02f)  // process noise: very low — heading changes slowly
#define COMPASS_UPDATE_MS             (100)

#define GRAVITY_MS2                   (9.806f)
#define KMH_TO_MS                     (1.0f / 3.6f)
#define MS_TO_KMH                     (3.6f)
#define US_TO_S                       (1000000.0f)
#define DEG_TO_RAD                    (0.01745329252f)

// Avoid stolen
#define DANGER_TILT_THRESHOLD_DEG     (30.0f)
#define DANGER_TILT_CONFIRM_MS        (800)
#define DANGER_MOTION_THRESHOLD_G     (0.18f)
#define DANGER_MOTION_CONFIRM_MS      (1200)
#define DANGER_VIBRATION_THRESHOLD_G  (0.35f)
#define DANGER_VIBRATION_WINDOW_MS    (3000)
#define DANGER_VIBRATION_COUNT_THRESH (5)

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  GPS_STATE_INVALID = 0,
  GPS_STATE_ACTIVE,
  GPS_STATE_FADING,
} gps_state_t;

typedef struct
{
  // GPS
  bool           is_new_gps_data_available;
  bsp_gps_data_t gps_data_buffer;
  size_t         last_gps_ms;
  float          velocity_gps;
  float          last_valid_lat;
  float          last_valid_lon;
  bool           has_last_gps_position;
  gps_state_t    gps_state;
  size_t         gps_lost_ms;

  // GPS reliability (Chiang 2013)
  float distance_ins;
  bool  gps_reliable;
  bool  is_new_gps_fix_this_cycle;

  // INS
  size_t              last_update_us;
  float               velocity_ins;
  float               velocity_out;
  float               acc_raw;
  float               acc_forward;
  float               offset_magnitude;
  float               distance_m;
  SimpleKalmanFilter *acc_kf_x;
  SimpleKalmanFilter *acc_kf_y;
  SimpleKalmanFilter *acc_kf_z;
  bool                acc_kf_init;

  SimpleKalmanFilter *gyro_kf_x;
  SimpleKalmanFilter *gyro_kf_y;
  SimpleKalmanFilter *gyro_kf_z;
  bool                gyro_kf_init;

  float roll_rad;
  float pitch_rad;
  // yaw is fusion_ctx.heading_deg (from compass, updated in sys_fusion_read_compass)

  float gyro_bias_x;  // rad/s
  float gyro_bias_y;  // rad/s
  float gyro_bias_z;  // rad/s

  // ZUPT
  bool     is_stationary;
  uint32_t stationary_time_ms;

  // Compass
  SimpleKalmanFilter *compass_kf_x;
  SimpleKalmanFilter *compass_kf_y;
  SimpleKalmanFilter *compass_kf_z;
  bool                compass_kf_init;
  size_t              compass_last_ms;
  float               heading_deg;
  const char         *direction_str;

  // Sensor ready flags
  bool compass_ready;
  bool acc_ready;
  bool gps_ready;
  bool is_offset_mag_ready;

  bool initialized;

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  // Debug data (send via mqtt) to include in payload for tuning and visualization
  float acc_raw_x;
  float acc_raw_y;
  float acc_raw_z;
  float acc_kf_filtered_x;
  float acc_kf_filtered_y;
  float acc_kf_filtered_z;
  float gyro_raw_x;
  float gyro_raw_y;
  float gyro_raw_z;
  float gyro_kf_filtered_x;
  float gyro_kf_filtered_y;
  float gyro_kf_filtered_z;
  float compass_raw_x;
  float compass_raw_y;
  float compass_raw_z;
  float compass_kf_filtered_x;
  float compass_kf_filtered_y;
  float compass_kf_filtered_z;
  float distance_gps;
#endif
} sys_fusion_context_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static sys_fusion_context_t fusion_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static float               sys_fusion_calculate_magnitude(float x, float y, float z);
static void                sys_fusion_calculate_offset_mag(void);
static void                sys_fusion_calibrate_gyro_bias(void);
static void                sys_fusion_update_ins_velocity(float dt);
static void                sys_fusion_update_gps_data(void);
static void                sys_fusion_update_gps_state(size_t current_ms);
static void                sys_fusion_detect_zupt(float accel_ms2, float dt);
static void                sys_fusion_compute_output_velocity(sys_fusion_data_t *data, float dt);
static void                sys_fusion_read_compass(sys_fusion_data_t *data, size_t current_ms);
static const char         *sys_fusion_deg_to_direction_str(float deg);
static void                sys_fusion_gps_callback(bsp_gps_data_t *gps_data);
static float               sys_fusion_caculate_dis_gps(float lat1, float lon1, float lat2, float lon2);
static SimpleKalmanFilter *kf_new(float e_mea, float e_est, float q);

/* Function definitions ----------------------------------------------- */
void sys_fusion_init(void)
{
  if (fusion_ctx.initialized)
    return;

  // 1. Reset context
  memset(&fusion_ctx, 0, sizeof(fusion_ctx));
  fusion_ctx.direction_str = "N";

  // 2. Create Kalman filters for accelerometer, gyro, compass
  fusion_ctx.acc_kf_x = kf_new(ACC_KF_E_MEA, ACC_KF_E_EST, ACC_KF_Q);
  fusion_ctx.acc_kf_y = kf_new(ACC_KF_E_MEA, ACC_KF_E_EST, ACC_KF_Q);
  fusion_ctx.acc_kf_z = kf_new(ACC_KF_E_MEA, ACC_KF_E_EST, ACC_KF_Q);
  if (fusion_ctx.acc_kf_x && fusion_ctx.acc_kf_y && fusion_ctx.acc_kf_z)
  {
    fusion_ctx.acc_kf_x->updateEstimate(0.0f);
    fusion_ctx.acc_kf_y->updateEstimate(0.0f);
    fusion_ctx.acc_kf_z->updateEstimate(1.0f);  // z at rest ≈ 1g
    fusion_ctx.acc_kf_init = true;
  }

  fusion_ctx.gyro_kf_x = kf_new(GYRO_KF_E_MEA, GYRO_KF_E_EST, GYRO_KF_Q);
  fusion_ctx.gyro_kf_y = kf_new(GYRO_KF_E_MEA, GYRO_KF_E_EST, GYRO_KF_Q);
  fusion_ctx.gyro_kf_z = kf_new(GYRO_KF_E_MEA, GYRO_KF_E_EST, GYRO_KF_Q);
  if (fusion_ctx.gyro_kf_x && fusion_ctx.gyro_kf_y && fusion_ctx.gyro_kf_z)
  {
    fusion_ctx.gyro_kf_x->updateEstimate(0.0f);
    fusion_ctx.gyro_kf_y->updateEstimate(0.0f);
    fusion_ctx.gyro_kf_z->updateEstimate(0.0f);
    fusion_ctx.gyro_kf_init = true;
  }

  fusion_ctx.compass_kf_x = kf_new(COMPASS_KF_E_MEA, COMPASS_KF_E_EST, COMPASS_KF_Q);
  fusion_ctx.compass_kf_y = kf_new(COMPASS_KF_E_MEA, COMPASS_KF_E_EST, COMPASS_KF_Q);
  fusion_ctx.compass_kf_z = kf_new(COMPASS_KF_E_MEA, COMPASS_KF_E_EST, COMPASS_KF_Q);
  if (fusion_ctx.compass_kf_x && fusion_ctx.compass_kf_y && fusion_ctx.compass_kf_z)
  {
    fusion_ctx.compass_kf_x->updateEstimate(0.0f);
    fusion_ctx.compass_kf_y->updateEstimate(0.0f);
    fusion_ctx.compass_kf_z->updateEstimate(0.0f);
    fusion_ctx.compass_kf_init = true;
  }

  // 3. Initialize sensors (ACC, GPS, Compass)
  LOG_DBG("Init ACC");
  if (bsp_acc_init() == STATUS_OK)
  {
    fusion_ctx.acc_ready = true;
    LOG_DBG("ACC OK");

    bsp_acc_raw_data_t init_acc = { 0 };
    if (bsp_acc_get_raw_data(&init_acc) == STATUS_OK && isfinite(init_acc.acc_x) && isfinite(init_acc.acc_y)
        && isfinite(init_acc.acc_z) && isfinite(init_acc.gyro_x) && isfinite(init_acc.gyro_y)
        && isfinite(init_acc.gyro_z))
    {
      // Re-seed with real first reading — only if all axes finite, avoids injecting NaN into KF
      fusion_ctx.acc_kf_x->updateEstimate(init_acc.acc_x);
      fusion_ctx.acc_kf_y->updateEstimate(init_acc.acc_y);
      fusion_ctx.acc_kf_z->updateEstimate(init_acc.acc_z);
      fusion_ctx.gyro_kf_x->updateEstimate(init_acc.gyro_x);
      fusion_ctx.gyro_kf_y->updateEstimate(init_acc.gyro_y);
      fusion_ctx.gyro_kf_z->updateEstimate(init_acc.gyro_z);

      fusion_ctx.roll_rad  = atan2f(init_acc.acc_y, init_acc.acc_z);
      fusion_ctx.pitch_rad = atan2f(-init_acc.acc_x, hypotf(init_acc.acc_y, init_acc.acc_z));
    }
    // Non-finite or failed read: KF keeps valid state from force-init above
  }
  else
  {
    LOG_ERR("ACC init failed");
  }

  LOG_DBG("Init GPS");
  if (bsp_gps_init(sys_fusion_gps_callback) == STATUS_OK)
  {
    fusion_ctx.gps_ready = true;
    LOG_DBG("GPS OK");
  }
  else
  {
    LOG_ERR("GPS init failed");
  }

  LOG_DBG("Init Compass");
  if (bsp_compass_init() == STATUS_OK)
  {
    fusion_ctx.compass_ready = true;
    LOG_DBG("Compass OK");
  }
  else
  {
    LOG_ERR("Compass init failed");
  }

  // 4. Calibrate accelerometer offset magnitude (for ZUPT)
  LOG_DBG("Calib acc offset. Device must be stationary");
  if (fusion_ctx.acc_ready)
  {
    sys_fusion_calculate_offset_mag();
    sys_fusion_calibrate_gyro_bias();
  }

  fusion_ctx.initialized = true;
}

status_function_t sys_fusion_process(sys_fusion_data_t *data)
{
  if (data == NULL || !fusion_ctx.initialized)
    return STATUS_ERROR;

  size_t current_time_us = micros();
  size_t current_time_ms = OS_GET_TICK();
  float  dt = (fusion_ctx.last_update_us == 0) ? 0.02f : (current_time_us - fusion_ctx.last_update_us) / US_TO_S;
  if (dt > 0.1f)  // Sample rates (50-100ms)
    dt = 0.1f;

  // 0. Reset new GPS fix flag
  fusion_ctx.is_new_gps_fix_this_cycle = false;

  // 1. Compass
  sys_fusion_read_compass(data, current_time_ms);

  // 2. Calculate Vins from Acc
  if (fusion_ctx.is_offset_mag_ready && fusion_ctx.acc_ready)
  {
    sys_fusion_update_ins_velocity(dt);
  }

  // 3. GPS state update
  sys_fusion_update_gps_data();
  sys_fusion_update_gps_state(current_time_ms);

  // 4. ZUPT
  sys_fusion_detect_zupt(fusion_ctx.acc_raw * GRAVITY_MS2, dt);

  // 5. Output velocity — complementary filter (INS + GPS)
  sys_fusion_compute_output_velocity(data, dt);

  // 6. Distance accumulation — single source: velocity_out
  if (fusion_ctx.is_offset_mag_ready && data->velocity_ms > GPS_SPEED_MIN_MS && dt > 0.0f)
  {
    fusion_ctx.distance_m += data->velocity_ms * dt;
  }

  if (fusion_ctx.velocity_ins != 0.0f)
  {
    LOG_DBG("Vins = %.2f m/s", fusion_ctx.velocity_ins);
  }
  if (fusion_ctx.velocity_gps != 0.0f)
  {
    LOG_DBG("Vgps = %.2f m/s", fusion_ctx.velocity_gps);
  }
  if (fusion_ctx.velocity_out != 0.0f)
  {
    LOG_DBG("Vout = %.2f m/s", fusion_ctx.velocity_out);
  }

  // 7. Finalize output
  data->distance_m             = fusion_ctx.distance_m;
  data->gps_position.latitude  = fusion_ctx.gps_data_buffer.latitude;
  data->gps_position.longitude = fusion_ctx.gps_data_buffer.longitude;

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  data->debug.acc_raw_x        = fusion_ctx.acc_raw_x;
  data->debug.acc_raw_y        = fusion_ctx.acc_raw_y;
  data->debug.acc_raw_z        = fusion_ctx.acc_raw_z;
  data->debug.acc_filter_x     = fusion_ctx.acc_kf_filtered_x;
  data->debug.acc_filter_y     = fusion_ctx.acc_kf_filtered_y;
  data->debug.acc_filter_z     = fusion_ctx.acc_kf_filtered_z;
  data->debug.gyro_raw_x       = fusion_ctx.gyro_raw_x;
  data->debug.gyro_raw_y       = fusion_ctx.gyro_raw_y;
  data->debug.gyro_raw_z       = fusion_ctx.gyro_raw_z;
  data->debug.gyro_filter_x    = fusion_ctx.gyro_kf_filtered_x;
  data->debug.gyro_filter_y    = fusion_ctx.gyro_kf_filtered_y;
  data->debug.gyro_filter_z    = fusion_ctx.gyro_kf_filtered_z;
  data->debug.compass_raw_x    = fusion_ctx.compass_raw_x;
  data->debug.compass_raw_y    = fusion_ctx.compass_raw_y;
  data->debug.compass_raw_z    = fusion_ctx.compass_raw_z;
  data->debug.compass_filter_x = fusion_ctx.compass_kf_filtered_x;
  data->debug.compass_filter_y = fusion_ctx.compass_kf_filtered_y;
  data->debug.compass_filter_z = fusion_ctx.compass_kf_filtered_z;
  data->debug.v_ins            = fusion_ctx.velocity_ins;
  data->debug.v_gps            = fusion_ctx.velocity_gps;
  data->debug.distance_ins     = fusion_ctx.distance_ins;
  data->debug.distance_gps     = fusion_ctx.distance_gps;
#endif

  fusion_ctx.last_update_us            = current_time_us;
  fusion_ctx.is_new_gps_data_available = false;

  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
static void sys_fusion_calculate_offset_mag(void)
{
  float sum = 0.0f;
  for (uint16_t i = 0; i < ACC_OFFSET_MAGNITUDE_SAMPLE; i++)
  {
    bsp_acc_raw_data_t d = { 0 };
    if (bsp_acc_get_raw_data(&d) == STATUS_OK)
    {
      sum += sys_fusion_calculate_magnitude((float) d.acc_x, (float) d.acc_y, (float) d.acc_z);
    }
    delay(5);  // CPU busy waiting
  }

  fusion_ctx.offset_magnitude    = sum / (float) ACC_OFFSET_MAGNITUDE_SAMPLE;
  fusion_ctx.is_offset_mag_ready = true;
  LOG_DBG("Offset calibrated: %.4f g", fusion_ctx.offset_magnitude);
  return;
}

static void sys_fusion_calibrate_gyro_bias(void)
{
  float    sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  uint16_t valid = 0;

  for (uint16_t i = 0; i < GYRO_BIAS_CALIB_SAMPLES; i++)
  {
    bsp_acc_raw_data_t d = { 0 };
    if (bsp_acc_get_raw_data(&d) == STATUS_OK)
    {
      sum_x += d.gyro_x * DEG_TO_RAD;
      sum_y += d.gyro_y * DEG_TO_RAD;
      sum_z += d.gyro_z * DEG_TO_RAD;
      valid++;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  if (valid > 0)
  {
    fusion_ctx.gyro_bias_x = sum_x / (float) valid;
    fusion_ctx.gyro_bias_y = sum_y / (float) valid;
    fusion_ctx.gyro_bias_z = sum_z / (float) valid;
    LOG_INF("[FUSION] Gyro bias: x=%.4f y=%.4f z=%.4f rad/s", fusion_ctx.gyro_bias_x, fusion_ctx.gyro_bias_y,
            fusion_ctx.gyro_bias_z);
  }
}

static float sys_fusion_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

static void sys_fusion_update_ins_velocity(float dt)
{
  bsp_acc_raw_data_t imu = { 0 };
  if (bsp_acc_get_raw_data(&imu) != STATUS_OK)
    return;

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.acc_raw_x  = imu.acc_x;
  fusion_ctx.acc_raw_y  = imu.acc_y;
  fusion_ctx.acc_raw_z  = imu.acc_z;
  fusion_ctx.gyro_raw_x = imu.gyro_x;
  fusion_ctx.gyro_raw_y = imu.gyro_y;
  fusion_ctx.gyro_raw_z = imu.gyro_z;
#endif

  // 1a. Kalman filter on gyro — replaces EMA(α=0.75)
  float gyro_kf_x, gyro_kf_y, gyro_kf_z;
  if (fusion_ctx.gyro_kf_init && isfinite(imu.gyro_x) && isfinite(imu.gyro_y) && isfinite(imu.gyro_z))
  {
    gyro_kf_x = fusion_ctx.gyro_kf_x->updateEstimate(imu.gyro_x);
    gyro_kf_y = fusion_ctx.gyro_kf_y->updateEstimate(imu.gyro_y);
    gyro_kf_z = fusion_ctx.gyro_kf_z->updateEstimate(imu.gyro_z);
  }
  else
  {
    // Non-finite input: keep last filtered value, don't corrupt KF state
    gyro_kf_x = isfinite(imu.gyro_x) ? imu.gyro_x : 0.0f;
    gyro_kf_y = isfinite(imu.gyro_y) ? imu.gyro_y : 0.0f;
    gyro_kf_z = isfinite(imu.gyro_z) ? imu.gyro_z : 0.0f;
  }

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.gyro_kf_filtered_x = gyro_kf_x;
  fusion_ctx.gyro_kf_filtered_y = gyro_kf_y;
  fusion_ctx.gyro_kf_filtered_z = gyro_kf_z;
#endif

  float acc_x, acc_y, acc_z;

  if (fusion_ctx.acc_kf_init && isfinite(imu.acc_x) && isfinite(imu.acc_y) && isfinite(imu.acc_z))
  {
    acc_x = fusion_ctx.acc_kf_x->updateEstimate(imu.acc_x);
    acc_y = fusion_ctx.acc_kf_y->updateEstimate(imu.acc_y);
    acc_z = fusion_ctx.acc_kf_z->updateEstimate(imu.acc_z);
  }
  else
  {
    acc_x = isfinite(imu.acc_x) ? imu.acc_x : 0.0f;
    acc_y = isfinite(imu.acc_y) ? imu.acc_y : 0.0f;
    acc_z = isfinite(imu.acc_z) ? imu.acc_z : 1.0f;  // fallback: 1g upright
  }

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.acc_kf_filtered_x = acc_x;
  fusion_ctx.acc_kf_filtered_y = acc_y;
  fusion_ctx.acc_kf_filtered_z = acc_z;
#endif

  // 2. Update attitude: roll, pitch
  float roll_acc  = atan2f(acc_y, acc_z);
  float pitch_acc = atan2f(-acc_x, hypotf(acc_y, acc_z));

  float gyro_x_rads = (gyro_kf_x * DEG_TO_RAD) - fusion_ctx.gyro_bias_x;
  float gyro_y_rads = (gyro_kf_y * DEG_TO_RAD) - fusion_ctx.gyro_bias_y;

  fusion_ctx.roll_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.roll_rad + gyro_x_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * roll_acc;
  fusion_ctx.pitch_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.pitch_rad + gyro_y_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * pitch_acc;

  if (fusion_ctx.is_stationary)
  {
    fusion_ctx.gyro_bias_x += GYRO_BIAS_ALPHA * ((gyro_kf_x * DEG_TO_RAD) - fusion_ctx.gyro_bias_x);
    fusion_ctx.gyro_bias_y += GYRO_BIAS_ALPHA * ((gyro_kf_y * DEG_TO_RAD) - fusion_ctx.gyro_bias_y);
    fusion_ctx.gyro_bias_z += GYRO_BIAS_ALPHA * ((gyro_kf_z * DEG_TO_RAD) - fusion_ctx.gyro_bias_z);
  }

  // 3. Body fram -> Navigation frame rotation (ZYX Euler, yaw from compass)
  float roll  = fusion_ctx.roll_rad;
  float pitch = fusion_ctx.pitch_rad;
  float yaw   = fusion_ctx.heading_deg * DEG_TO_RAD;

  float sin_roll  = sinf(roll);
  float cos_roll  = cosf(roll);
  float sin_pitch = sinf(pitch);
  float cos_pitch = cosf(pitch);
  float sin_yaw   = sinf(yaw);
  float cos_yaw   = cosf(yaw);

  float abx = acc_x * GRAVITY_MS2;
  float aby = acc_y * GRAVITY_MS2;
  float abz = acc_z * GRAVITY_MS2;

  float acc_north = cos_pitch * cos_yaw * abx + (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw) * aby
                    + (cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw) * abz;
  float acc_east = cos_pitch * sin_yaw * abx + (sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw) * aby
                   + (cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw) * abz;
  // acc_d (vertical): acc_d = -sin_pitch*abx + sin_roll*cos_pitch*aby + cos_roll*cos_pitch*abz - GRAVITY_MS2 (not
  // needed here)

  // 4. Forward projection onto heading direction
  float acc_forward      = acc_north * cos_yaw + acc_east * sin_yaw;
  float mag_g            = hypotf(hypotf(acc_x, acc_y), acc_z);
  fusion_ctx.acc_raw     = mag_g - fusion_ctx.offset_magnitude;
  fusion_ctx.acc_forward = acc_forward;
  bool gravity_leak      = (fabsf(acc_forward) > ACC_FORWARD_MAX_MS2) && (fabsf(mag_g - 1.0f) < 0.25f);

  // 5. INS velocity integration
  if (!gravity_leak && fabsf(acc_forward) > ACC_THRESHOLD_MS2)
  {
    fusion_ctx.velocity_ins += acc_forward * dt;
  }
  else
  {
    float decay;
    if (fusion_ctx.is_stationary)
      decay = INS_DECAY_STOPPING;  // Hard brake: stationary confirmed
    else if (fusion_ctx.gps_state == GPS_STATE_FADING)
      decay = INS_DECAY_GPS_LOST;  // Soft brake: GPS signal lost
    else
      decay = INS_DECAY_NORMAL;  // Gentle drift compensation while moving

    fusion_ctx.velocity_ins *= powf(decay, dt / 0.02f);
  }

  if (fusion_ctx.velocity_ins < 0.0f)
    fusion_ctx.velocity_ins = 0.0f;

  // 6. Accumulate INS distance for GPS reliability check (Chiang 2013)
  if (fusion_ctx.velocity_ins > GPS_SPEED_MIN_MS)
    fusion_ctx.distance_ins += fusion_ctx.velocity_ins * dt;
}

static void sys_fusion_update_gps_data(void)
{
  if (!fusion_ctx.gps_ready)
  {
    if (bsp_gps_init(sys_fusion_gps_callback) == STATUS_OK)
    {
      fusion_ctx.gps_ready = true;
    }
    else
    {
      return;
    }
  }

  if (!fusion_ctx.is_new_gps_data_available || !fusion_ctx.gps_data_buffer.location_valid)
  {
    return;
  }

  float raw_speed = (float) fusion_ctx.gps_data_buffer.speed_kmph * KMH_TO_MS;
  bool  is_gps_data_ok =
    (fusion_ctx.gps_data_buffer.hdop < GPS_HDOP_MAX) && (fusion_ctx.gps_data_buffer.satellites >= GPS_SATELLITES_MIN);

  if (!is_gps_data_ok || raw_speed < GPS_SPEED_MIN_MS)
  {
    fusion_ctx.velocity_gps = 0.0f;
    fusion_ctx.velocity_ins *= (1.0f - GPS_ANCHOR_RATE);
  }
  else
  {
    fusion_ctx.velocity_gps = GPS_EMA_ALPHA * raw_speed + (1.0f - GPS_EMA_ALPHA) * fusion_ctx.velocity_gps;
    fusion_ctx.velocity_ins =
      (1.0f - GPS_ANCHOR_RATE) * fusion_ctx.velocity_ins + GPS_ANCHOR_RATE * fusion_ctx.velocity_gps;
    fusion_ctx.is_new_gps_fix_this_cycle = true;  // FIX: mark fresh GPS fix for CF
  }

  fusion_ctx.last_gps_ms = OS_GET_TICK();

  if (is_gps_data_ok)
  {
    float lat = (float) fusion_ctx.gps_data_buffer.latitude;
    float lon = (float) fusion_ctx.gps_data_buffer.longitude;

    if (fusion_ctx.has_last_gps_position)
    {
      float distance_gps = sys_fusion_caculate_dis_gps(fusion_ctx.last_valid_lat, fusion_ctx.last_valid_lon, lat, lon);

#if (DEVICE_FUSION_DEBUG_MODE == 1)
      fusion_ctx.distance_gps = distance_gps;
#endif

      // GPS reliability check (Chiang 2013):
      // z_r = |d_INS - d_GPS|; reject GPS if residual exceeds threshold
      // TODO: need update position-based reliability check instead of distance-based, otherwise GPS will be rejected
      float z_r = fabsf(fusion_ctx.distance_ins - distance_gps);
      if ((z_r < GPS_RELIABILITY_THRESHOLD_M) && (fusion_ctx.distance_ins != 0.0f))
      {
        fusion_ctx.gps_reliable = true;
      }
      else
      {
        fusion_ctx.gps_reliable = false;
        LOG_DBG("GPS rejected: z_r=%.1fm (ins=%.1fm gps=%.1fm)", z_r, fusion_ctx.distance_ins, distance_gps);
      }
    }
    else
    {
      fusion_ctx.gps_reliable = true;
    }

    fusion_ctx.distance_ins          = 0.0f;
    fusion_ctx.last_valid_lat        = lat;
    fusion_ctx.last_valid_lon        = lon;
    fusion_ctx.has_last_gps_position = true;
  }
}

static void sys_fusion_update_gps_state(size_t current_ms)
{
  bool gps_recently_updated =
    (fusion_ctx.last_gps_ms > 0) && ((current_ms - fusion_ctx.last_gps_ms) < GPS_VALID_TIMEOUT_MS);

  switch (fusion_ctx.gps_state)
  {
  case GPS_STATE_INVALID:
    if (gps_recently_updated && fusion_ctx.velocity_gps > 0.0f)
    {
      fusion_ctx.gps_state = GPS_STATE_ACTIVE;
      LOG_DBG("GPS: INVALID -> ACTIVE");
    }
    break;

  case GPS_STATE_ACTIVE:
    if (!gps_recently_updated)
    {
      fusion_ctx.gps_state   = GPS_STATE_FADING;
      fusion_ctx.gps_lost_ms = current_ms;
      LOG_DBG("GPS: ACTIVE -> FADING");
    }
    break;

  case GPS_STATE_FADING:
    if (gps_recently_updated && fusion_ctx.velocity_gps > 0.0f)
    {
      fusion_ctx.gps_state = GPS_STATE_ACTIVE;
      LOG_DBG("GPS: FADING -> ACTIVE");
    }
    else if ((current_ms - fusion_ctx.gps_lost_ms) >= GPS_FADE_TIMEOUT_MS)
    {
      fusion_ctx.gps_state             = GPS_STATE_INVALID;
      fusion_ctx.velocity_gps          = 0.0f;
      fusion_ctx.gps_reliable          = false;
      fusion_ctx.distance_ins          = 0.0f;
      fusion_ctx.has_last_gps_position = false;
      LOG_DBG("GPS: FADING -> INVALID");
    }
    break;

  default: fusion_ctx.gps_state = GPS_STATE_INVALID; break;
  }
}

static void sys_fusion_detect_zupt(float accel_ms2, float dt)
{
  if (fabsf(accel_ms2) < ZUPT_ACC_THRESHOLD)
  {
    if (!fusion_ctx.is_stationary)
    {
      fusion_ctx.is_stationary      = true;
      fusion_ctx.stationary_time_ms = 0;
    }
    else
    {
      fusion_ctx.stationary_time_ms += (uint32_t) (dt * 1000.0f);
    }

    if (fusion_ctx.stationary_time_ms >= ZUPT_TIME_THRESHOLD_MS)
    {
      fusion_ctx.velocity_ins = 0.0f;
      fusion_ctx.velocity_gps = 0.0f;
    }
  }
  else
  {
    fusion_ctx.is_stationary      = false;
    fusion_ctx.stationary_time_ms = 0;
  }
}

static void sys_fusion_compute_output_velocity(sys_fusion_data_t *data, float dt)
{
  bool use_gps =
    fusion_ctx.is_new_gps_fix_this_cycle && (fusion_ctx.gps_state == GPS_STATE_ACTIVE) && fusion_ctx.gps_reliable;
  float v_gps_eff = use_gps ? fusion_ctx.velocity_gps : 0.0f;

  float dt_gps = dt;
  if (use_gps && fusion_ctx.last_gps_ms > 0)
  {
    size_t now_ms  = OS_GET_TICK();
    float  elapsed = (now_ms - fusion_ctx.last_gps_ms) / 1000.0f;
    // Clamp to [dt, 1.0s] — reject absurd values
    if (elapsed > dt && elapsed < 1.0f)
      dt_gps = elapsed;
  }

  // Memory + INS weights use dt_fusion (every cycle)
  float denom_ins = 1.0f + CF_WC * dt;
  float gamma     = 1.0f / denom_ins;
  float alpha     = dt / denom_ins;

  // GPS weight uses dt_gps — proportional to the GPS update interval
  // so total GPS energy per unit time stays constant regardless of rate.
  float beta = use_gps ? (CF_WC * dt_gps / (1.0f + CF_WC * dt_gps)) : 0.0f;

  // When GPS fires, reduce memory weight to preserve unity gain:
  // gamma_adj + alpha + beta = 1
  float gamma_adj = use_gps ? (1.0f - alpha - beta) : gamma;
  if (gamma_adj < 0.0f)
    gamma_adj = 0.0f;

  fusion_ctx.velocity_out = gamma_adj * fusion_ctx.velocity_out + alpha * fusion_ctx.velocity_ins + beta * v_gps_eff;

  if (fusion_ctx.velocity_out < 0.0f)
    fusion_ctx.velocity_out = 0.0f;

  float ins_blend;
  if (fusion_ctx.gps_state == GPS_STATE_ACTIVE && fusion_ctx.gps_reliable)
    ins_blend = INS_BLEND_ACTIVE;
  else if (fusion_ctx.gps_state == GPS_STATE_FADING)
    ins_blend = INS_BLEND_FADING;
  else
    ins_blend = INS_BLEND_INVALID;

  fusion_ctx.velocity_out = (1.0f - ins_blend) * fusion_ctx.velocity_out + ins_blend * fusion_ctx.velocity_ins;

  if (fusion_ctx.velocity_out < 0.0f)
    fusion_ctx.velocity_out = 0.0f;

  data->velocity_ms  = fusion_ctx.velocity_out;
  data->velocity_kmh = fusion_ctx.velocity_out * MS_TO_KMH;
}

static const char *s_direction_strings[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

static const char *sys_fusion_deg_to_direction_str(float deg)
{
  uint8_t index;
  if (deg < 22.5f || deg >= 337.5f)
    index = 0;
  else if (deg < 67.5f)
    index = 1;
  else if (deg < 112.5f)
    index = 2;
  else if (deg < 157.5f)
    index = 3;
  else if (deg < 202.5f)
    index = 4;
  else if (deg < 247.5f)
    index = 5;
  else if (deg < 292.5f)
    index = 6;
  else
    index = 7;
  return s_direction_strings[index];
}

static void sys_fusion_read_compass(sys_fusion_data_t *data, size_t current_ms)
{
  // Always return the latest known heading, even when skipping this cycle
  data->heading_deg   = fusion_ctx.heading_deg;
  data->direction_str = fusion_ctx.direction_str;

  if (!fusion_ctx.compass_ready)
    return;
  if ((current_ms - fusion_ctx.compass_last_ms) < COMPASS_UPDATE_MS)
    return;
  fusion_ctx.compass_last_ms = current_ms;

  bsp_compass_raw_data_t raw_data;
  if (bsp_compass_read_raw(&raw_data) != STATUS_OK)
  {
    LOG_ERR("Read compass fail");
    return;
  }

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.compass_raw_x = (float) raw_data.raw_x;
  fusion_ctx.compass_raw_y = (float) raw_data.raw_y;
  fusion_ctx.compass_raw_z = (float) raw_data.raw_z;
#endif

  float rx   = (float) raw_data.raw_x;
  float ry   = (float) raw_data.raw_y;
  float rz   = (float) raw_data.raw_z;
  float kf_x = isfinite(rx) ? fusion_ctx.compass_kf_x->updateEstimate(rx) : fusion_ctx.compass_kf_x->getFilteredValue();
  float kf_y = isfinite(ry) ? fusion_ctx.compass_kf_y->updateEstimate(ry) : fusion_ctx.compass_kf_y->getFilteredValue();
  float kf_z = isfinite(rz) ? fusion_ctx.compass_kf_z->updateEstimate(rz) : fusion_ctx.compass_kf_z->getFilteredValue();

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.compass_kf_filtered_x = kf_x;
  fusion_ctx.compass_kf_filtered_y = kf_y;
  fusion_ctx.compass_kf_filtered_z = kf_z;
#endif

  float heading_rad = atan2f(kf_y, kf_x);
  float heading_deg = heading_rad * 180.0f / (float) M_PI;
  if (heading_deg < 0.0f)
    heading_deg += 360.0f;

  fusion_ctx.heading_deg   = heading_deg;
  fusion_ctx.direction_str = sys_fusion_deg_to_direction_str(heading_deg);

  data->heading_deg   = fusion_ctx.heading_deg;
  data->direction_str = fusion_ctx.direction_str;
}

static void sys_fusion_gps_callback(bsp_gps_data_t *gps_data)
{
  if (gps_data == NULL)
    return;
  fusion_ctx.gps_data_buffer           = *gps_data;
  fusion_ctx.is_new_gps_data_available = true;
  fusion_ctx.last_gps_ms               = OS_GET_TICK();
}

static float sys_fusion_caculate_dis_gps(float lat1, float lon1, float lat2, float lon2)
{
  const float R    = 6371000.0f;
  float       dlat = (lat2 - lat1) * (float) M_PI / 180.0f;
  float       dlon = (lon2 - lon1) * (float) M_PI / 180.0f;
  float       a =
    sinf(dlat / 2.0f) * sinf(dlat / 2.0f)
    + cosf(lat1 * (float) M_PI / 180.0f) * cosf(lat2 * (float) M_PI / 180.0f) * sinf(dlon / 2.0f) * sinf(dlon / 2.0f);
  return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

void sys_fusion_detect_danger_motion(sys_fusion_danger_motion_flag_t *out_flags)
{
  static size_t  tilt_start_ms   = 0;
  static size_t  motion_start_ms = 0;
  static size_t  vibration_ms    = 0;
  static uint8_t vibration_count = 0;

  sys_fusion_danger_motion_flag_t flags = SYS_FUSION_DANGER_MOTION_NONE;
  bsp_acc_raw_data_t              raw;
  size_t                          now = OS_GET_TICK();

  if (bsp_acc_get_raw_data(&raw) != STATUS_OK)
  {
    return;
  }

  float magnitude = sqrtf(raw.acc_x * raw.acc_x + raw.acc_y * raw.acc_y + raw.acc_z * raw.acc_z);

  if (magnitude > 0.1f)
  {
    float tilt_angle_deg = acosf(fabsf(raw.acc_z) / magnitude) * (180.0f / (float) M_PI);

    if (tilt_angle_deg > DANGER_TILT_THRESHOLD_DEG)
    {
      if (tilt_start_ms == 0)
      {
        tilt_start_ms = now;
      }
      else if ((now - tilt_start_ms) >= DANGER_TILT_CONFIRM_MS)
      {
        flags = SYS_FUSION_DANGER_MOTION_TILT;
        return;
      }
    }
    else
    {
      tilt_start_ms = 0;
    }
  }

  float lateral_g = sqrtf(raw.acc_x * raw.acc_x + raw.acc_y * raw.acc_y);
  if (lateral_g > DANGER_MOTION_THRESHOLD_G)
  {
    if (motion_start_ms == 0)
    {
      motion_start_ms = now;
    }
    else if ((now - motion_start_ms) >= DANGER_MOTION_CONFIRM_MS)
    {
      flags = SYS_FUSION_DANGER_MOTION_MOVING;
    }
  }
  else
  {
    motion_start_ms = 0;
  }

  if ((now - vibration_ms) >= DANGER_VIBRATION_WINDOW_MS)
  {
    vibration_ms    = now;
    vibration_count = 0;
  }

  if (magnitude > DANGER_VIBRATION_THRESHOLD_G + 1.0f)
  {
    vibration_count++;
  }

  if (vibration_count >= DANGER_VIBRATION_COUNT_THRESH)
  {
    flags = SYS_FUSION_DANGER_MOTION_VIBRATION;
  }

  if (out_flags != NULL)
  {
    *out_flags = flags;
  }

  return;
}

static SimpleKalmanFilter *kf_new(float e_mea, float e_est, float q)
{
  void *mem = calloc(1, sizeof(SimpleKalmanFilter));
  if (mem == NULL)
    return NULL;
  return new (mem) SimpleKalmanFilter(e_mea, e_est, q);
}

/* End of file -------------------------------------------------------- */