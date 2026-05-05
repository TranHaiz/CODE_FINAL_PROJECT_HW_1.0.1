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
LOG_MODULE_REGISTER(sys_fusion, LOG_LEVEL_SYS_FUSION)

#define DEMO_VEHICLE                (true)
#define DEMO_WALKING                (false)

// Accelerometer parameters
#define ACC_EMA_ALPHA               (0.4f)
#define ACC_EMA_ALPHA_FAST          (0.70f)
#define ACC_EMA_ALPHA_MEDIUM        (0.50f)
#define ACC_EMA_ALPHA_SLOW          (0.18f)
#define ACC_THRESHOLD_MS2           (0.02f)  // Dead-band to gate INS integration (m/s²)
#define ACC_OFFSET_MAGNITUDE_SAMPLE (200)

#define ACC_FWD_DELTA_FAST          (0.60f)
#define ACC_FWD_DELTA_MEDIUM        (0.30f)
#define ACC_FWD_QUIET_LIMIT         (0.15f)

// Active-motion threshold — used to suppress GPS anchoring during transients,
// because GPS speed lags real motion by ~1s during fast accel/decel.
#define ACTIVE_MOTION_TH_MS2        (0.3f)
#define GPS_ANCHOR_TRANSIENT_SCALE  (0.2f)  // Multiplier on anchor when active motion

// Attitude complementary filter (gyro + accelerometer)
#define ATTITUDE_GYRO_WEIGHT        (0.90f)
#define GYRO_BIAS_CALIB_SAMPLES     (200)
#define GYRO_BIAS_ALPHA             (0.01f)
#define ACC_FORWARD_MAX_MS2         (3.0f)

// Velocity complementary filter crossover frequency (rad/s)  [Zhao 2020]
// Higher = faster GPS tracking; lower = smoother INS-dominant output
#define CF_WC                       (0.8f)

#if (DEMO_VEHICLE)
#define ZUPT_ACC_THRESHOLD          (0.10f)
#define ZUPT_TIME_THRESHOLD_MS      (1200)
#define INS_DECAY_NORMAL            (0.990f)
#define INS_DECAY_STOPPING          (0.87f)
#define INS_DECAY_GPS_LOST          (0.95f)
#define GPS_SPEED_MIN_MS            (0.8f)
#define GPS_ANCHOR_RATE             (0.8f)
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

#define GPS_HDOP_MAX                       (3.0f)
#define GPS_SATELLITES_MIN                 (4)
#define GPS_EMA_ALPHA                      (0.65f)  // Slightly more responsive than 0.6
#define GPS_MAX_STEP_M                     (50.0f)
#define GPS_VALID_TIMEOUT_MS               (2000)
#define GPS_FADE_TIMEOUT_MS                (1000)

#define COMPASS_EMA_ALPHA                  (0.15f)
#define COMPASS_UPDATE_MS                  (100)

#define GRAVITY_MS2                        (9.806f)
#define KMH_TO_MS                          (1.0f / 3.6f)
#define MS_TO_KMH                          (3.6f)
#define US_TO_S                            (1000000.0f)
#define DEG_TO_RAD                         (0.01745329252f)

// Avoid stolen
#define DANGER_TILT_THRESHOLD_DEG          (30.0f)  // If device tilted >30° for certain time
#define DANGER_TILT_CONFIRM_MS             (800)    // Must be tilted for at least 800ms to confirm
#define DANGER_MOTION_THRESHOLD_G          (0.18f)  // If strong motion >0.18g for certain time
#define DANGER_MOTION_CONFIRM_MS           (1200)   // Must have strong motion for at least 1200ms to confirm
#define DANGER_VIBRATION_THRESHOLD_G       (0.35f)  // If vibration magnitude >0.35g for certain time
#define DANGER_VIBRATION_WINDOW_MS         (3000)   // Count how many strong vibration events in this rolling window
#define DANGER_VIBRATION_COUNT_THRESH      (5)  // If strong vibration events exceed this count in the window, confirm danger

#define DANGER_TILT_THRESHOLD_DEG_HIGH     (10.0f)
#define DANGER_TILT_CONFIRM_MS_HIGH        (200)
#define DANGER_MOTION_THRESHOLD_G_HIGH     (0.10f)
#define DANGER_MOTION_CONFIRM_MS_HIGH      (400)
#define DANGER_VIBRATION_THRESHOLD_G_HIGH  (0.10f)
#define DANGER_VIBRATION_WINDOW_MS_HIGH    (2000)
#define DANGER_VIBRATION_COUNT_THRESH_HIGH (2)

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
  float distance_ins;  // INS-accumulated distance between GPS updates
  bool  gps_reliable;
  bool  is_new_gps_fix_this_cycle;

  // INS
  size_t last_update_us;
  float  velocity_ins;  // Raw INS integrated velocity
  float  velocity_out;  // Complementary filter output velocity
  float  acc_raw;       // Net dynamic acc magnitude (g) — used for ZUPT
  float  acc_forward;   // Forward acceleration after body→nav projection (m/s²)
  float  prev_acc_forward;
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

  float gyro_bias_x;  // rad/s
  float gyro_bias_y;  // rad/s
  float gyro_bias_z;  // rad/s

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

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  // Debug snapshots — last raw readings and derived values
  float debug_acc_raw_x;
  float debug_acc_raw_y;
  float debug_acc_raw_z;
  float debug_gyro_raw_x;
  float debug_gyro_raw_y;
  float debug_gyro_raw_z;
  float debug_gyro_ema_x;
  float debug_gyro_ema_y;
  float debug_gyro_ema_z;
  bool  debug_gyro_ema_init;
  float debug_compass_raw_x;
  float debug_compass_raw_y;
  float debug_compass_raw_z;
  float debug_distance_gps;  // Last GPS step distance (haversine, m)
#endif
} sys_fusion_context_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static sys_fusion_context_t fusion_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static float       sys_fusion_calculate_magnitude(float x, float y, float z);
static void        sys_fusion_calculate_offset_mag(void);
static void        sys_fusion_calibrate_gyro_bias(void);
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

  fusion_ctx.is_new_gps_fix_this_cycle = false;

  // 1. Compass, sample rates: COMPASS_UPDATE_MS
  sys_fusion_read_compass(data, current_time_ms);

  // 2. Caculate Vins from Acc
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

  // 6. INS-only distance fallback when GPS unavailable
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

  // 7. Finalize output
  data->distance_m             = fusion_ctx.distance_m;
  data->gps_position.latitude  = fusion_ctx.gps_data_buffer.latitude;
  data->gps_position.longitude = fusion_ctx.gps_data_buffer.longitude;

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  data->debug.acc_raw_x        = fusion_ctx.debug_acc_raw_x;
  data->debug.acc_raw_y        = fusion_ctx.debug_acc_raw_y;
  data->debug.acc_raw_z        = fusion_ctx.debug_acc_raw_z;
  data->debug.acc_filter_x     = fusion_ctx.acc_ema_x;
  data->debug.acc_filter_y     = fusion_ctx.acc_ema_y;
  data->debug.acc_filter_z     = fusion_ctx.acc_ema_z;
  data->debug.gyro_raw_x       = fusion_ctx.debug_gyro_raw_x;
  data->debug.gyro_raw_y       = fusion_ctx.debug_gyro_raw_y;
  data->debug.gyro_raw_z       = fusion_ctx.debug_gyro_raw_z;
  data->debug.gyro_filter_x    = fusion_ctx.debug_gyro_ema_x;
  data->debug.gyro_filter_y    = fusion_ctx.debug_gyro_ema_y;
  data->debug.gyro_filter_z    = fusion_ctx.debug_gyro_ema_z;
  data->debug.compass_raw_x    = fusion_ctx.debug_compass_raw_x;
  data->debug.compass_raw_y    = fusion_ctx.debug_compass_raw_y;
  data->debug.compass_raw_z    = fusion_ctx.debug_compass_raw_z;
  data->debug.compass_filter_x = fusion_ctx.compass_ema_x;
  data->debug.compass_filter_y = fusion_ctx.compass_ema_y;
  data->debug.compass_filter_z = fusion_ctx.compass_ema_z;
  data->debug.v_ins            = fusion_ctx.velocity_ins;
  data->debug.v_gps            = fusion_ctx.velocity_gps;
  data->debug.distance_ins     = fusion_ctx.distance_ins;
  data->debug.distance_gps     = fusion_ctx.debug_distance_gps;
#endif

  fusion_ctx.last_update_us            = current_time_us;
  fusion_ctx.is_new_gps_data_available = false;

  return STATUS_OK;
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

  bool    is_high_danger   = (g_device_info.danger_level == DEVICE_DANGER_LEVEL_HIGH);
  float   tilt_thresh_deg  = is_high_danger ? DANGER_TILT_THRESHOLD_DEG_HIGH : DANGER_TILT_THRESHOLD_DEG;
  size_t  tilt_conf_ms     = is_high_danger ? DANGER_TILT_CONFIRM_MS_HIGH : DANGER_TILT_CONFIRM_MS;
  float   motion_thresh_g  = is_high_danger ? DANGER_MOTION_THRESHOLD_G_HIGH : DANGER_MOTION_THRESHOLD_G;
  size_t  motion_conf_ms   = is_high_danger ? DANGER_MOTION_CONFIRM_MS_HIGH : DANGER_MOTION_CONFIRM_MS;
  float   vib_thresh_g     = is_high_danger ? DANGER_VIBRATION_THRESHOLD_G_HIGH : DANGER_VIBRATION_THRESHOLD_G;
  size_t  vib_window_ms    = is_high_danger ? DANGER_VIBRATION_WINDOW_MS_HIGH : DANGER_VIBRATION_WINDOW_MS;
  uint8_t vib_count_thresh = is_high_danger ? DANGER_VIBRATION_COUNT_THRESH_HIGH : DANGER_VIBRATION_COUNT_THRESH;

  if (bsp_acc_get_raw_data(&raw) != STATUS_OK)
  {
    return;
  }

  float magnitude = sqrtf(raw.acc_x * raw.acc_x + raw.acc_y * raw.acc_y + raw.acc_z * raw.acc_z);

  if (magnitude > 0.1f)
  {
    float tilt_angle_deg = acosf(fabsf(raw.acc_z) / magnitude) * (180.0f / (float) M_PI);

    if (tilt_angle_deg > tilt_thresh_deg)
    {
      if (tilt_start_ms == 0)
      {
        tilt_start_ms = now;
      }
      else if ((now - tilt_start_ms) >= tilt_conf_ms)
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
  if (lateral_g > motion_thresh_g)
  {
    if (motion_start_ms == 0)
    {
      motion_start_ms = now;
    }
    else if ((now - motion_start_ms) >= motion_conf_ms)
    {
      flags = SYS_FUSION_DANGER_MOTION_MOVING;
    }
  }
  else
  {
    motion_start_ms = 0;
  }

  if ((now - vibration_ms) >= vib_window_ms)
  {
    vibration_ms    = now;
    vibration_count = 0;
  }

  if (magnitude > vib_thresh_g + 1.0f)
  {
    vibration_count++;
  }

  if (vibration_count >= vib_count_thresh)
  {
    flags = SYS_FUSION_DANGER_MOTION_VIBRATION;
  }

  if (out_flags != NULL)
  {
    *out_flags = flags;
  }

  return;
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
  fusion_ctx.debug_acc_raw_x  = imu.acc_x;
  fusion_ctx.debug_acc_raw_y  = imu.acc_y;
  fusion_ctx.debug_acc_raw_z  = imu.acc_z;
  fusion_ctx.debug_gyro_raw_x = imu.gyro_x;
  fusion_ctx.debug_gyro_raw_y = imu.gyro_y;
  fusion_ctx.debug_gyro_raw_z = imu.gyro_z;

  if (!fusion_ctx.debug_gyro_ema_init)
  {
    fusion_ctx.debug_gyro_ema_x    = imu.gyro_x;
    fusion_ctx.debug_gyro_ema_y    = imu.gyro_y;
    fusion_ctx.debug_gyro_ema_z    = imu.gyro_z;
    fusion_ctx.debug_gyro_ema_init = true;
  }
  else
  {
    fusion_ctx.debug_gyro_ema_x = ACC_EMA_ALPHA * imu.gyro_x + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.debug_gyro_ema_x;
    fusion_ctx.debug_gyro_ema_y = ACC_EMA_ALPHA * imu.gyro_y + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.debug_gyro_ema_y;
    fusion_ctx.debug_gyro_ema_z = ACC_EMA_ALPHA * imu.gyro_z + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.debug_gyro_ema_z;
  }
#endif

  // 1. Acc EMA filter - 3 axes (giữ nguyên cho attitude)
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

  float acc_x = fusion_ctx.acc_ema_x;
  float acc_y = fusion_ctx.acc_ema_y;
  float acc_z = fusion_ctx.acc_ema_z;

  // 2. Update attitude: roll, pitch
  float roll_acc  = atan2f(imu.acc_y, imu.acc_z);
  float pitch_acc = atan2f(-imu.acc_x, hypotf(imu.acc_y, imu.acc_z));

  float gyro_x_rads = (imu.gyro_x * DEG_TO_RAD) - fusion_ctx.gyro_bias_x;
  float gyro_y_rads = (imu.gyro_y * DEG_TO_RAD) - fusion_ctx.gyro_bias_y;

  fusion_ctx.roll_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.roll_rad + gyro_x_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * roll_acc;
  fusion_ctx.pitch_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.pitch_rad + gyro_y_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * pitch_acc;

  if (fusion_ctx.is_stationary)
  {
    fusion_ctx.gyro_bias_x += GYRO_BIAS_ALPHA * ((imu.gyro_x * DEG_TO_RAD) - fusion_ctx.gyro_bias_x);
    fusion_ctx.gyro_bias_y += GYRO_BIAS_ALPHA * ((imu.gyro_y * DEG_TO_RAD) - fusion_ctx.gyro_bias_y);
    fusion_ctx.gyro_bias_z += GYRO_BIAS_ALPHA * ((imu.gyro_z * DEG_TO_RAD) - fusion_ctx.gyro_bias_z);
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

  // 4. Forward projection + Adaptive Filter
  float acc_forward_raw = acc_north * cos_yaw + acc_east * sin_yaw;

  float mag_g        = hypotf(hypotf(acc_x, acc_y), acc_z);
  fusion_ctx.acc_raw = mag_g - fusion_ctx.offset_magnitude;

  float alpha = ACC_EMA_ALPHA;
  float delta = fabsf(acc_forward_raw - fusion_ctx.prev_acc_forward);
  if (delta > ACC_FWD_DELTA_FAST)
  {
    alpha = ACC_EMA_ALPHA_FAST;
  }
  else if (delta > ACC_FWD_DELTA_MEDIUM)
  {
    alpha = ACC_EMA_ALPHA_MEDIUM;
  }
  else if (fabsf(acc_forward_raw) < ACC_FWD_QUIET_LIMIT)
  {
    alpha = ACC_EMA_ALPHA_SLOW;
  }
  fusion_ctx.acc_forward      = alpha * acc_forward_raw + (1.0f - alpha) * fusion_ctx.prev_acc_forward;
  fusion_ctx.prev_acc_forward = fusion_ctx.acc_forward;

  // 5. INS velocity integration
  bool gravity_leak = (fabsf(acc_forward_raw) > ACC_FORWARD_MAX_MS2) && (fabsf(mag_g - 1.0f) < 0.25f);
  if (!gravity_leak && fabsf(fusion_ctx.acc_forward) > ACC_THRESHOLD_MS2)
  {
    fusion_ctx.velocity_ins += fusion_ctx.acc_forward * dt;
  }
  else
  {
    float decay;
    if (fusion_ctx.is_stationary)
      decay = INS_DECAY_STOPPING;
    else if (fusion_ctx.gps_state == GPS_STATE_FADING)
      decay = INS_DECAY_GPS_LOST;
    else
      decay = INS_DECAY_NORMAL;

    fusion_ctx.velocity_ins *= powf(decay, dt / 0.02f);
  }

  if (fusion_ctx.velocity_ins < 0.0f)
    fusion_ctx.velocity_ins = 0.0f;

  // 6. Accumulate INS distance
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
    if (fusion_ctx.is_stationary)
      fusion_ctx.velocity_ins *= (1.0f - GPS_ANCHOR_RATE);
  }
  else
  {
    fusion_ctx.velocity_gps = GPS_EMA_ALPHA * raw_speed + (1.0f - GPS_EMA_ALPHA) * fusion_ctx.velocity_gps;
    float anchor            = GPS_ANCHOR_RATE;
    if (fabsf(fusion_ctx.acc_forward) > ACTIVE_MOTION_TH_MS2)
      anchor *= GPS_ANCHOR_TRANSIENT_SCALE;

    fusion_ctx.velocity_ins              = (1.0f - anchor) * fusion_ctx.velocity_ins + anchor * fusion_ctx.velocity_gps;
    fusion_ctx.is_new_gps_fix_this_cycle = true;  // FIX: mark fresh GPS fix for CF
  }

  fusion_ctx.last_gps_ms = OS_GET_TICK();

  if (is_gps_data_ok)
  {
    float lat = (float) fusion_ctx.gps_data_buffer.latitude;
    float lon = (float) fusion_ctx.gps_data_buffer.longitude;

    if (fusion_ctx.has_last_gps_position)
    {
      float distance_gps = sys_fusion_haversine_m(fusion_ctx.last_valid_lat, fusion_ctx.last_valid_lon, lat, lon);

#if (DEVICE_FUSION_DEBUG_MODE == 1)
      fusion_ctx.debug_distance_gps = distance_gps;
#endif

      // GPS reliability check (Chiang 2013):
      // z_r = |d_INS - d_GPS|; reject GPS if residual exceeds threshold
      float z_r = fabsf(fusion_ctx.distance_ins - distance_gps);
      if (z_r < GPS_RELIABILITY_THRESHOLD_M)
      {
        fusion_ctx.gps_reliable = true;
        if (distance_gps < GPS_MAX_STEP_M && fusion_ctx.velocity_gps > GPS_SPEED_MIN_MS)
          fusion_ctx.distance_m += distance_gps;
      }
      else
      {
        fusion_ctx.gps_reliable = false;
        LOG_WRN("GPS rejected: z_r=%.1fm (ins=%.1fm gps=%.1fm)", z_r, fusion_ctx.distance_ins, distance_gps);
      }
    }
    else
    {
      // First valid fix — no INS reference yet, trust GPS
      fusion_ctx.gps_reliable = true;
    }

    // Reset INS distance accumulator for next GPS interval
    fusion_ctx.distance_ins = 0.0f;

    fusion_ctx.last_valid_lat        = lat;
    fusion_ctx.last_valid_lon        = lon;
    fusion_ctx.has_last_gps_position = true;
  }
}

static void sys_fusion_update_gps_state(size_t current_ms)
{
  bool gps_recently_updated =
    (fusion_ctx.last_gps_ms > 0) && ((current_ms - fusion_ctx.last_gps_ms) < GPS_VALID_TIMEOUT_MS);

  bool is_gps_data_ok = fusion_ctx.gps_data_buffer.location_valid && (fusion_ctx.gps_data_buffer.hdop < GPS_HDOP_MAX)
                        && (fusion_ctx.gps_data_buffer.satellites >= GPS_SATELLITES_MIN);

  switch (fusion_ctx.gps_state)
  {
  case GPS_STATE_INVALID:
    if (gps_recently_updated && is_gps_data_ok)
    {
      fusion_ctx.gps_state = GPS_STATE_ACTIVE;
      LOG_DBG("GPS: INVALID -> ACTIVE");
    }
    break;

  case GPS_STATE_ACTIVE:
    if (!gps_recently_updated || !is_gps_data_ok)
    {
      fusion_ctx.gps_state   = GPS_STATE_FADING;
      fusion_ctx.gps_lost_ms = current_ms;
      LOG_DBG("GPS: ACTIVE -> FADING");
    }
    break;

  case GPS_STATE_FADING:
    if (gps_recently_updated && is_gps_data_ok)
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
      // Snap CF output too — without this, vout would coast down via CF for ~3*tau
      fusion_ctx.velocity_out = 0.0f;
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
  // GPS branch: only inject when a fresh fix arrived this cycle
  bool use_gps =
    fusion_ctx.is_new_gps_fix_this_cycle && (fusion_ctx.gps_state == GPS_STATE_ACTIVE) && fusion_ctx.gps_reliable;
  float v_gps_eff = use_gps ? fusion_ctx.velocity_gps : 0.0f;

  // dt_gps: time since last GPS fix (used to scale GPS weight correctly)
  // Falls back to dt when no GPS so the expression stays well-formed.
  float dt_gps = dt;
  if (use_gps && fusion_ctx.last_gps_ms > 0)
  {
    size_t now_ms  = OS_GET_TICK();
    float  elapsed = (now_ms - fusion_ctx.last_gps_ms) / 1000.0f;
    // Clamp to [dt, 1.0s] — reject absurd values
    if (elapsed > dt && elapsed < 1.0f)
      dt_gps = elapsed;
  }

  float denom_ins = 1.0f + CF_WC * dt;
  float gamma     = 1.0f / denom_ins;
  float alpha     = (CF_WC * dt) / denom_ins;
  float beta      = use_gps ? (CF_WC * dt_gps / (1.0f + CF_WC * dt_gps)) : 0.0f;
  float gamma_adj = use_gps ? (1.0f - alpha - beta) : gamma;

  if (gamma_adj < 0.0f)
    gamma_adj = 0.0f;

  fusion_ctx.velocity_out = gamma_adj * fusion_ctx.velocity_out + alpha * fusion_ctx.velocity_ins + beta * v_gps_eff;

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
  fusion_ctx.debug_compass_raw_x = (float) raw_data.raw_x;
  fusion_ctx.debug_compass_raw_y = (float) raw_data.raw_y;
  fusion_ctx.debug_compass_raw_z = (float) raw_data.raw_z;
#endif

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

  float mag_x = fusion_ctx.compass_ema_x;
  float mag_y = fusion_ctx.compass_ema_y;
  float mag_z = fusion_ctx.compass_ema_z;

  float cos_roll  = cosf(fusion_ctx.roll_rad);
  float sin_roll  = sinf(fusion_ctx.roll_rad);
  float cos_pitch = cosf(fusion_ctx.pitch_rad);
  float sin_pitch = sinf(fusion_ctx.pitch_rad);

  float Xh = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z * cos_roll * sin_pitch;
  float Yh = mag_y * cos_roll - mag_z * sin_roll;

  float heading_rad = atan2f(Yh, Xh);
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