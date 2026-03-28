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

#include <math.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_fusion, LOG_LEVEL_WARN)

// #define DEMO_VEHICLE (1)
#define DEMO_WALKING                (1)

// Accelerometer parameters
#define ACC_EMA_ALPHA               (0.3f)   // Per-axis EMA before body→nav rotation
#define ACC_THRESHOLD_MS2           (0.05f)  // Dead-band to gate INS integration (m/s²)
#define ACC_OFFSET_MAGNITUDE_SAMPLE (200)

// Attitude complementary filter (gyro + accelerometer)
// ATTITUDE_GYRO_WEIGHT: fraction from gyro integration each step
// 0.98 = mostly gyro dynamics, 2% accelerometer drift-correction per step
#define ATTITUDE_GYRO_WEIGHT        (0.98f)

// Velocity complementary filter crossover frequency (rad/s)  [Zhao 2020]
// Higher = faster GPS tracking; lower = smoother INS-dominant output
#define CF_WC                       (1.0f)

#if defined(DEMO_VEHICLE)
#define ZUPT_ACC_THRESHOLD          (0.03f)
#define ZUPT_TIME_THRESHOLD_MS      (1000)
#define INS_DECAY_NORMAL            (0.9998f)  // Very slow decay while riding
#define INS_DECAY_STOPPING          (0.94f)    // Fast decay when stopped
#define INS_DECAY_GPS_LOST          (0.97f)    // Medium decay during GPS fade-out
#define GPS_SPEED_MIN_MS            (0.6f)     // 2.2 km/h
#define GPS_ANCHOR_RATE             (0.7f)     // Strong GPS anchor outdoors
#define GPS_RELIABILITY_THRESHOLD_M (20.0f)    // Max |d_INS - d_GPS| before GPS rejected

#elif defined(DEMO_WALKING)
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

#define GPS_HDOP_MAX         (3.0f)
#define GPS_SATELLITES_MIN   (4)
#define GPS_EMA_ALPHA        (0.6f)
#define GPS_MAX_STEP_M       (50.0f)
#define GPS_VALID_TIMEOUT_MS (2000)
#define GPS_FADE_TIMEOUT_MS  (1000)

#define COMPASS_EMA_ALPHA    (0.15f)
#define COMPASS_UPDATE_MS    (100)

#define GRAVITY_MS2          (9.806f)
#define KMH_TO_MS            (1.0f / 3.6f)
#define MS_TO_KMH            (3.6f)
#define US_TO_S              (1000000.0f)
#define DEG_TO_RAD           (0.01745329252f)

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
  float ins_dist_since_gps;  // INS-accumulated distance between GPS updates
  bool  gps_reliable;

  // INS
  size_t last_update_us;
  float  velocity_ins;  // Raw INS integrated velocity
  float  velocity_out;  // Complementary filter output velocity
  float  acc_raw;       // Net dynamic acc magnitude (g) — used for ZUPT
  float  acc_forward;   // Forward acceleration after body→nav projection (m/s²)
  float  offset_magnitude;
  float  distance_m;

  // Acc per-axis EMA (filtered before rotation)
  float acc_ema_x;
  float acc_ema_y;
  float acc_ema_z;
  bool  acc_ema_init;

  // Attitude — continuously updated via gyro + acc complementary filter
  float roll_rad;
  float pitch_rad;
  // yaw is fusion_ctx.heading_deg (from compass, updated in sys_fusion_read_compass)

  // ZUPT
  bool     is_stationary;
  uint32_t stationary_time_ms;

  // Compass
  float       compass_ema_x;
  float       compass_ema_y;
  float       compass_ema_z;
  size_t      compass_last_ms;
  bool        compass_filter_init;
  float       heading_deg;
  const char *direction_str;

  // Sensor ready flags
  bool compass_ready;
  bool acc_ready;
  bool gps_ready;
  bool is_offset_mag_ready;

  bool initialized;
} sys_fusion_context_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static sys_fusion_context_t fusion_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static float       sys_fusion_calculate_magnitude(float x, float y, float z);
static void        sys_fusion_calculate_offset_mag(void);
static void        sys_fusion_update_ins_velocity(float dt);
static void        sys_fusion_update_gps_data(void);
static void        sys_fusion_update_gps_state(size_t current_ms);
static void        sys_fusion_detect_zupt(float accel_ms2, float dt);
static void        sys_fusion_compute_output_velocity(sys_fusion_data_t *data, float dt);
static void        sys_fusion_read_compass(sys_fusion_data_t *data, size_t current_ms);
static const char *sys_fusion_deg_to_direction_str(float deg);
static void        sys_fusion_gps_callback(bsp_gps_data_t *gps_data);
static float       sys_fusion_haversine_m(float lat1, float lon1, float lat2, float lon2);

/* Function definitions ----------------------------------------------- */
void sys_fusion_init(void)
{
  if (fusion_ctx.initialized)
    return;

  memset(&fusion_ctx, 0, sizeof(fusion_ctx));
  fusion_ctx.direction_str = "N";

  LOG_DBG("Init ACC");
  if (bsp_acc_init() == STATUS_OK)
  {
    fusion_ctx.acc_ready = true;
    LOG_DBG("ACC OK");

    bsp_acc_raw_data_t init_acc = { 0 };
    if (bsp_acc_get_raw_data(&init_acc) == STATUS_OK)
    {
      fusion_ctx.acc_ema_x    = init_acc.acc_x;
      fusion_ctx.acc_ema_y    = init_acc.acc_y;
      fusion_ctx.acc_ema_z    = init_acc.acc_z;
      fusion_ctx.acc_ema_init = true;

      fusion_ctx.roll_rad  = atan2f(init_acc.acc_y, init_acc.acc_z);
      fusion_ctx.pitch_rad = atan2f(-init_acc.acc_x, hypotf(init_acc.acc_y, init_acc.acc_z));
    }
  }

  LOG_DBG("Init GPS");
  if (bsp_gps_init(sys_fusion_gps_callback) == STATUS_OK)
  {
    fusion_ctx.gps_ready = true;
    LOG_DBG("GPS OK");
  }

  LOG_DBG("Init Compass");
  if (bsp_compass_init() == STATUS_OK)
  {
    fusion_ctx.compass_ready = true;
    LOG_DBG("Compass OK");
  }

  LOG_DBG("Calib acc offset. Device must be stationary");
  if (fusion_ctx.acc_ready)
  {
    sys_fusion_calculate_offset_mag();
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

  // 1. Caculate Vins from Acc
  if (fusion_ctx.is_offset_mag_ready && fusion_ctx.acc_ready)
  {
    sys_fusion_update_ins_velocity(dt);
  }

  // 2. ZUPT for Vins
  sys_fusion_detect_zupt(fusion_ctx.acc_raw * GRAVITY_MS2, dt);

  // 3. GPS state update
  sys_fusion_update_gps_data();
  sys_fusion_update_gps_state(current_time_ms);

  // 4. Output velocity — complementary filter (INS + GPS)
  sys_fusion_compute_output_velocity(data, dt);

  // 5. INS-only distance fallback when GPS unavailable
  if (fusion_ctx.gps_state == GPS_STATE_INVALID && fusion_ctx.is_offset_mag_ready && dt > 0.0f
      && data->velocity_ms > GPS_SPEED_MIN_MS)
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

  // 6. Compass — rate-limited to COMPASS_UPDATE_MS, always copies cached heading
  sys_fusion_read_compass(data, current_time_ms);

  // 7. Finalize output
  data->distance_m             = fusion_ctx.distance_m;
  data->gps_position.latitude  = fusion_ctx.gps_data_buffer.latitude;
  data->gps_position.longitude = fusion_ctx.gps_data_buffer.longitude;

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

static float sys_fusion_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

/**
 * @brief Update INS velocity from IMU data.
 *
 * Steps:
 *   1. Per-axis EMA filter on acc (preserves direction for rotation)
 *   2. Attitude update — complementary filter: gyro integration + acc correction
 *   3. Body → Navigation frame rotation (ZYX Euler, yaw from cached compass)
 *   4. Project nav-frame acc onto forward (heading) direction
 *   5. INS velocity integration with threshold dead-band and decay
 *   6. Accumulate INS distance for GPS reliability check (Chiang 2013)
 */
static void sys_fusion_update_ins_velocity(float dt)
{
  bsp_acc_raw_data_t imu = { 0 };
  if (bsp_acc_get_raw_data(&imu) != STATUS_OK)
    return;

  // --- Step 1: Per-axis EMA filter ---
  if (!fusion_ctx.acc_ema_init)
  {
    fusion_ctx.acc_ema_x    = imu.acc_x;
    fusion_ctx.acc_ema_y    = imu.acc_y;
    fusion_ctx.acc_ema_z    = imu.acc_z;
    fusion_ctx.acc_ema_init = true;
  }
  else
  {
    fusion_ctx.acc_ema_x = ACC_EMA_ALPHA * imu.acc_x + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.acc_ema_x;
    fusion_ctx.acc_ema_y = ACC_EMA_ALPHA * imu.acc_y + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.acc_ema_y;
    fusion_ctx.acc_ema_z = ACC_EMA_ALPHA * imu.acc_z + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.acc_ema_z;
  }

  float ax = fusion_ctx.acc_ema_x;
  float ay = fusion_ctx.acc_ema_y;
  float az = fusion_ctx.acc_ema_z;

  // --- Step 2: Attitude update (roll, pitch) ---
  // Accelerometer provides tilt reference (valid when near-static)
  float roll_acc  = atan2f(ay, az);
  float pitch_acc = atan2f(-ax, hypotf(ay, az));

  // Gyroscope gives dynamic rotation rate (dps → rad/s)
  float gyro_x_rads = imu.gyro_x * DEG_TO_RAD;
  float gyro_y_rads = imu.gyro_y * DEG_TO_RAD;

  // Complementary filter: gyro tracks dynamics, acc corrects long-term drift
  fusion_ctx.roll_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.roll_rad + gyro_x_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * roll_acc;
  fusion_ctx.pitch_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.pitch_rad + gyro_y_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * pitch_acc;

  // --- Step 3: Body → Navigation frame (ZYX Euler rotation matrix) ---
  // Yaw from cached compass heading — updated on its own rate in sys_fusion_read_compass
  float roll  = fusion_ctx.roll_rad;
  float pitch = fusion_ctx.pitch_rad;
  float yaw   = fusion_ctx.heading_deg * DEG_TO_RAD;

  float sr = sinf(roll);
  float cr = cosf(roll);
  float sp = sinf(pitch);
  float cp = cosf(pitch);
  float sy = sinf(yaw);
  float cy = cosf(yaw);

  // Specific force in body frame (m/s²)
  float abx = ax * GRAVITY_MS2;
  float aby = ay * GRAVITY_MS2;
  float abz = az * GRAVITY_MS2;

  // Rotate to NED navigation frame
  float acc_n = cp * cy * abx + (sr * sp * cy - cr * sy) * aby + (cr * sp * cy + sr * sy) * abz;
  float acc_e = cp * sy * abx + (sr * sp * sy + cr * cy) * aby + (cr * sp * sy - sr * cy) * abz;
  // acc_d (vertical): acc_d = -sp*abx + sr*cp*aby + cr*cp*abz - GRAVITY_MS2 (not needed here)

  // --- Step 4: Project onto forward (heading) direction ---
  float acc_forward = acc_n * cy + acc_e * sy;

  // Net dynamic acc magnitude for ZUPT (subtract static gravity baseline)
  float mag_g        = hypotf(hypotf(ax, ay), az);
  fusion_ctx.acc_raw = mag_g - fusion_ctx.offset_magnitude;

  fusion_ctx.acc_forward = acc_forward;

  // --- Step 5: INS velocity integration ---
  if (fabsf(acc_forward) > ACC_THRESHOLD_MS2)
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

  // --- Step 6: Accumulate INS distance for GPS reliability check (Chiang 2013) ---
  if (fusion_ctx.velocity_ins > GPS_SPEED_MIN_MS)
    fusion_ctx.ins_dist_since_gps += fusion_ctx.velocity_ins * dt;
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
  }

  fusion_ctx.last_gps_ms = OS_GET_TICK();

  if (is_gps_data_ok)
  {
    float lat = (float) fusion_ctx.gps_data_buffer.latitude;
    float lon = (float) fusion_ctx.gps_data_buffer.longitude;

    if (fusion_ctx.has_last_gps_position)
    {
      float d_gps = sys_fusion_haversine_m(fusion_ctx.last_valid_lat, fusion_ctx.last_valid_lon, lat, lon);

      // GPS reliability check (Chiang 2013):
      // z_r = |d_INS - d_GPS|; reject GPS if residual exceeds threshold
      float z_r = fabsf(fusion_ctx.ins_dist_since_gps - d_gps);
      if (z_r < GPS_RELIABILITY_THRESHOLD_M)
      {
        fusion_ctx.gps_reliable = true;
        if (d_gps < GPS_MAX_STEP_M && fusion_ctx.velocity_gps > GPS_SPEED_MIN_MS)
          fusion_ctx.distance_m += d_gps;
      }
      else
      {
        fusion_ctx.gps_reliable = false;
        LOG_WRN("GPS rejected: z_r=%.1fm (ins=%.1fm gps=%.1fm)", z_r, fusion_ctx.ins_dist_since_gps, d_gps);
      }
    }
    else
    {
      // First valid fix — no INS reference yet, trust GPS
      fusion_ctx.gps_reliable = true;
    }

    // Reset INS distance accumulator for next GPS interval
    fusion_ctx.ins_dist_since_gps = 0.0f;

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
      fusion_ctx.gps_state    = GPS_STATE_INVALID;
      fusion_ctx.velocity_gps = 0.0f;
      fusion_ctx.gps_reliable = false;
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

/**
 * @brief Compute final output velocity using complementary filter (Zhao 2020).
 *
 * v̂(k) = γ·v̂(k-1) + α·v_ins(k) + β·v_gps(k)
 *
 * Where:
 *   γ = 1 / (1 + wc·dt)          — memory weight
 *   α = dt / (1 + wc·dt)         — INS weight
 *   β = wc·dt / (1 + wc·dt)      — GPS weight
 *
 * GPS velocity is only used when the GPS state is ACTIVE and the most recent
 * GPS fix passed the reliability check (Chiang 2013 residual test).
 */
static void sys_fusion_compute_output_velocity(sys_fusion_data_t *data, float dt)
{
  // Only trust GPS when active and reliability check passed
  float v_gps_eff = 0.0f;
  if (fusion_ctx.gps_state == GPS_STATE_ACTIVE && fusion_ctx.gps_reliable)
    v_gps_eff = fusion_ctx.velocity_gps;

  float denom = 1.0f + CF_WC * dt;
  float gamma = 1.0f / denom;
  float alpha = dt / denom;
  float beta  = CF_WC * dt / denom;

  fusion_ctx.velocity_out = gamma * fusion_ctx.velocity_out + alpha * fusion_ctx.velocity_ins + beta * v_gps_eff;

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

/**
 * @brief Read compass and update heading_deg / direction_str.
 *
 * Rate-limited to COMPASS_UPDATE_MS. Always copies the cached heading into
 * data at the top so callers always get a valid (possibly slightly stale) value.
 *
 * NOTE: This function must NOT be called from sys_fusion_update_ins_velocity.
 * The yaw used for body→nav rotation reads fusion_ctx.heading_deg directly,
 * avoiding double-consumption of the rate-limit window.
 */
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

  if (!fusion_ctx.compass_filter_init)
  {
    fusion_ctx.compass_ema_x       = (float) raw_data.raw_x;
    fusion_ctx.compass_ema_y       = (float) raw_data.raw_y;
    fusion_ctx.compass_ema_z       = (float) raw_data.raw_z;
    fusion_ctx.compass_filter_init = true;
  }
  else
  {
    fusion_ctx.compass_ema_x =
      COMPASS_EMA_ALPHA * (float) raw_data.raw_x + (1.0f - COMPASS_EMA_ALPHA) * fusion_ctx.compass_ema_x;
    fusion_ctx.compass_ema_y =
      COMPASS_EMA_ALPHA * (float) raw_data.raw_y + (1.0f - COMPASS_EMA_ALPHA) * fusion_ctx.compass_ema_y;
    fusion_ctx.compass_ema_z =
      COMPASS_EMA_ALPHA * (float) raw_data.raw_z + (1.0f - COMPASS_EMA_ALPHA) * fusion_ctx.compass_ema_z;
  }

  float heading_rad = atan2f(fusion_ctx.compass_ema_y, fusion_ctx.compass_ema_x);
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

static float sys_fusion_haversine_m(float lat1, float lon1, float lat2, float lon2)
{
  const float R    = 6371000.0f;
  float       dlat = (lat2 - lat1) * (float) M_PI / 180.0f;
  float       dlon = (lon2 - lon1) * (float) M_PI / 180.0f;
  float       a =
    sinf(dlat / 2.0f) * sinf(dlat / 2.0f)
    + cosf(lat1 * (float) M_PI / 180.0f) * cosf(lat2 * (float) M_PI / 180.0f) * sinf(dlon / 2.0f) * sinf(dlon / 2.0f);
  return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

/* End of file -------------------------------------------------------- */
