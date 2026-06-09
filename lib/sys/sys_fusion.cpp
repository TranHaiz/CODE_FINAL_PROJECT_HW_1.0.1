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
#include "sys_fusion_log.h"
#include "sys_led.h"

#include <TinyGPSPlus.h>
#include <math.h>

#include "device_config.h"

#if (DEVICE_FUSION_ALGO == DEVICE_FUSION_ALGO_V2)

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_fusion, LOG_LEVEL_SYS_FUSION)

#define DEMO_VEHICLE                (true)
#define DEMO_WALKING                (false)

// Accelerometer parameters
#define ACC_EMA_ALPHA               (0.077f)  // EMA fallback path (TC ~60ms at 5ms dt)
#define DBG_GYRO_EMA_ALPHA          (0.3f)    // debug visualization only — algo uses raw gyro
#define ACC_THRESHOLD_MS2           (0.02f)   // Dead-band to gate INS integration (m/s²)
#define ACC_OFFSET_MAGNITUDE_SAMPLE (200)
#define ACC_SAMPLING_INTERVAL_MS    (50.0f)
#define ACC_DENTA_SEC               (ACC_SAMPLING_INTERVAL_MS / 1000.0f)

// Active-motion threshold — used to suppress GPS anchoring during transients,
// because GPS speed lags real motion by ~1s during fast accel/decel.
#define ACTIVE_MOTION_TH_MS2        (0.3f)
#define GPS_ANCHOR_TRANSIENT_SCALE  (0.2f)  // Multiplier on anchor when active motion

// Attitude complementary filter (gyro + accelerometer)
#define ATTITUDE_GYRO_WEIGHT        (0.9975f)
#define GYRO_BIAS_CALIB_SAMPLES     (200)
#define GYRO_BIAS_ALPHA             (0.01f)
#define ACC_FORWARD_MAX_MS2         (3.0f)

#define YAW_GYRO_WEIGHT             (0.98f)
#define YAW_GYRO_WEIGHT_STATIONARY  (0.85f)
#define YAW_ROTATING_TH_DEGPS       (10.0f)
#define YAW_OFFSET_DEG              (0.0f)

// Velocity complementary filter crossover frequency (rad/s)  [Zhao 2020]
// Higher = faster GPS tracking; lower = smoother INS-dominant output
#define CF_WC                       (1.0f)
#define CF_WC_TRANSIENT             (4.0f)
#define CF_WC_TRANSIENT_TH_MS2      (1.5f)
#define CF_TRANSIENT_AGREE_MS       (2.0f)
// Stop-response shaping: make Vout decay quicker near standstill.
#define CF_WC_STOPPING              (6.0f)
#define VEL_NEAR_ZERO_MS            (0.18f)
#define VEL_SETTLE_BAND_MS          (0.12f)

// Output velocity safety anchor — bounds vout drift when vins is suspect
#define VOUT_ANCHOR_NONE            (0)  // no extra anchor (baseline)
#define VOUT_ANCHOR_SOFT            (1)  // continuous slow pull vout => vgps (TC ~2.5s)
#define VOUT_ANCHOR_SNAP            (2)  // hard snap when |vout-vgps| exceeds threshold
#define VOUT_ANCHOR_MODE            (VOUT_ANCHOR_SOFT)
#define VOUT_ANCHOR_SOFT_RATE       (0.002f)  // per-cycle pull rate (TC ~2.5s @ 200Hz)
#define VOUT_SNAP_TH_MS             (3.0f)    // m/s — snap when |vout-vgps| > 10.8 km/h

#if (DEMO_VEHICLE)
#define ZUPT_ACC_THRESHOLD          (0.15f)
#define ZUPT_TIME_THRESHOLD_MS      (400)
#define INS_DECAY_NORMAL            (0.990f)
#define INS_DECAY_STOPPING          (0.75f)
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
#define GPS_MAX_STEP_M                     (50.0f)
#define GPS_VALID_TIMEOUT_MS               (2000)
#define GPS_FADE_TIMEOUT_MS                (1000)

#define COMPASS_EMA_ALPHA                  (0.15f)
#define COMPASS_UPDATE_MS                  (100)

// Butterworth 2nd-order lowpass for ACC: fc=8 Hz, fs=200 Hz
// wn=tan(pi*8/200)=0.12683, D=1+sqrt(2)*wn+wn^2
#define BW_B0                              (0.013450f)
#define BW_B1                              (0.026899f)
#define BW_B2                              (0.013450f)
#define BW_A1                              (-1.646225f)
#define BW_A2                              (0.699928f)

#define COMPASS_BW_B0                      (0.067455f)
#define COMPASS_BW_B1                      (0.134911f)
#define COMPASS_BW_B2                      (0.067455f)
#define COMPASS_BW_A1                      (-1.142981f)
#define COMPASS_BW_A2                      (0.412776f)

#define GRAVITY_MS2                        (9.806f)
#define KMH_TO_MS                          (1.0f / 3.6f)
#define MS_TO_KMH                          (3.6f)
#define US_TO_S                            (1000000.0f)
#define DEG_TO_RAD                         (0.01745329252f)

// Avoid stolen
#define DANGER_TILT_THRESHOLD_DEG          (45.0f)  // If device tilted >30° for certain time
#define DANGER_TILT_CONFIRM_MS             (1500)   // Must be tilted for at least 800ms to confirm
#define DANGER_MOTION_THRESHOLD_G          (0.3f)   // If strong motion >0.18g for certain time
#define DANGER_MOTION_CONFIRM_MS           (2000)   // Must have strong motion for at least 1200ms to confirm
#define DANGER_VIBRATION_THRESHOLD_G       (0.5f)   // If vibration magnitude >0.35g for certain time
#define DANGER_VIBRATION_WINDOW_MS         (3000)   // Count how many strong vibration events in this rolling window
#define DANGER_VIBRATION_COUNT_THRESH      (6)  // If strong vibration events exceed this count in the window, confirm danger

#define DANGER_TILT_THRESHOLD_DEG_HIGH     (10.0f)
#define DANGER_TILT_CONFIRM_MS_HIGH        (200)
#define DANGER_MOTION_THRESHOLD_G_HIGH     (0.10f)
#define DANGER_MOTION_CONFIRM_MS_HIGH      (400)
#define DANGER_VIBRATION_THRESHOLD_G_HIGH  (0.10f)
#define DANGER_VIBRATION_WINDOW_MS_HIGH    (2000)
#define DANGER_VIBRATION_COUNT_THRESH_HIGH (2)

// Include AFTER all compile-time #defines — sys_fusion_tune.h #undef's them and rebinds
// the same names to live struct fields when DEVICE_FUSION_TUNING_MODE_ENABLED is set.
#include "sys_fusion_tune.h"

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  GPS_STATE_INVALID = 0,
  GPS_STATE_ACTIVE,
  GPS_STATE_FADING,
} gps_state_t;

#if (DEVICE_FUSION_ACC_FILTER == DEVICE_FUSION_FILTER_BTW) || (DEVICE_FUSION_COMPASS_FILTER == DEVICE_FUSION_FILTER_BTW)
typedef struct
{
  float x1, x2;
  float y1, y2;
} biquad_state_t;
typedef struct
{
  float b0, b1, b2;
  float a1, a2;
} biquad_coefs_t;
#endif

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
  float  offset_magnitude;
  float  distance_m;
  float  distance_gap_ins;    // INS distance counted while GPS not authoritative; netted out on GPS accept
  float  distance_ins_total;  // cumulative INS-only distance (never reset, debug)
  float  distance_gps_total;  // cumulative GPS haversine (never reset, debug)

  // Acc per-axis filter (EMA or Butterworth, selected by DEVICE_FUSION_ACC_FILTER)
  float acc_ema_x;
  float acc_ema_y;
  float acc_ema_z;
  bool  acc_ema_init;

#if (DEVICE_FUSION_ACC_FILTER == DEVICE_FUSION_FILTER_BTW)
  biquad_state_t bw_x;
  biquad_state_t bw_y;
  biquad_state_t bw_z;
  bool           bw_init;
#endif

#if (DEVICE_FUSION_COMPASS_FILTER == DEVICE_FUSION_FILTER_BTW)
  biquad_state_t compass_bw_x;
  biquad_state_t compass_bw_y;
  biquad_state_t compass_bw_z;
#endif

  // Attitude — gyro+acc CF for roll/pitch, gyro_z+compass CF for yaw (heading_deg)
  float roll_rad;
  float pitch_rad;
  bool  yaw_init;
  float latest_gyro_z_dps;

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
static bool        sys_fusion_preprocess_data(size_t current_ms);
static bool        sys_fusion_read_imu(bsp_acc_raw_data_t *imu);
static void        sys_fusion_update_attitude(const bsp_acc_raw_data_t *imu, bool compass_fresh, float dt);
static void        sys_fusion_update_ins_velocity(float dt);
static void        sys_fusion_update_gps_data(void);
static void        sys_fusion_update_gps_state(size_t current_ms);
static bool        sys_fusion_detect_zupt(float accel_ms2, float dt);
static void        sys_fusion_compute_output_velocity(sys_fusion_data_t *data, float dt);
static const char *sys_fusion_deg_to_direction_str(float deg);
static void        sys_fusion_gps_callback(bsp_gps_data_t *gps_data);
static float       sys_fusion_haversine_m(float lat1, float lon1, float lat2, float lon2);
static float       sys_fusion_wrap_to_180(float deg);

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
    // Initial roll/pitch seeded in sys_fusion_calculate_offset_mag() below
  }
  else
  {
    sys_led_write_event(SYS_LED_EVT_ERROR_IMU);
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
    sys_led_write_event(SYS_LED_EVT_ERROR_GPS);
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
  if (dt > ACC_DENTA_SEC)
    dt = ACC_DENTA_SEC;

  fusion_ctx.is_new_gps_fix_this_cycle = false;

  // Carry over last known heading for cycles where the attitude step is skipped
  data->heading_deg   = fusion_ctx.heading_deg;
  data->direction_str = fusion_ctx.direction_str;

  // 1. Read + preprocess sensors
  bool               compass_fresh = sys_fusion_preprocess_data(current_time_ms);
  bsp_acc_raw_data_t imu           = { 0 };
  bool               imu_ok = (fusion_ctx.is_offset_mag_ready && fusion_ctx.acc_ready) && sys_fusion_read_imu(&imu);

  // 2. Attitude CF, body to nav, Vins/Dins
  if (imu_ok)
  {
    sys_fusion_update_attitude(&imu, compass_fresh, dt);
    data->heading_deg   = fusion_ctx.heading_deg;
    data->direction_str = fusion_ctx.direction_str;
    sys_fusion_update_ins_velocity(dt);
  }

  // 3. GPS state update
  sys_fusion_update_gps_data();
  sys_fusion_update_gps_state(current_time_ms);

  // 4. ZUPT
  bool zupt_locked = sys_fusion_detect_zupt(fusion_ctx.acc_raw * GRAVITY_MS2, dt);

  // 5. Output velocity
  if (zupt_locked)
  {
    data->velocity_ms  = 0.0f;
    data->velocity_kmh = 0.0f;
  }
  else
  {
    sys_fusion_compute_output_velocity(data, dt);
  }

  // 6. INS distance fallback when GPS not authoritative; tracked in distance_gap_ins to net out on accept
  bool gps_authoritative = (fusion_ctx.gps_state == GPS_STATE_ACTIVE) && fusion_ctx.gps_reliable;
  if (!gps_authoritative && fusion_ctx.is_offset_mag_ready && dt > 0.0f && data->velocity_ms > GPS_SPEED_MIN_MS)
  {
    float d = data->velocity_ms * dt;
    fusion_ctx.distance_m += d;
    fusion_ctx.distance_gap_ins += d;
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
  data->distance_m = fusion_ctx.distance_m;
  if (fusion_ctx.has_last_gps_position)
  {
    data->gps_position.latitude  = fusion_ctx.last_valid_lat;
    data->gps_position.longitude = fusion_ctx.last_valid_lon;
  }
  else
  {
    data->gps_position.latitude  = 0.0;
    data->gps_position.longitude = 0.0;
  }

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  data->debug.acc_raw_x          = fusion_ctx.debug_acc_raw_x;
  data->debug.acc_raw_y          = fusion_ctx.debug_acc_raw_y;
  data->debug.acc_raw_z          = fusion_ctx.debug_acc_raw_z;
  data->debug.acc_filter_x       = fusion_ctx.acc_ema_x;
  data->debug.acc_filter_y       = fusion_ctx.acc_ema_y;
  data->debug.acc_filter_z       = fusion_ctx.acc_ema_z;
  data->debug.gyro_raw_x         = fusion_ctx.debug_gyro_raw_x;
  data->debug.gyro_raw_y         = fusion_ctx.debug_gyro_raw_y;
  data->debug.gyro_raw_z         = fusion_ctx.debug_gyro_raw_z;
  data->debug.gyro_filter_x      = fusion_ctx.debug_gyro_ema_x;
  data->debug.gyro_filter_y      = fusion_ctx.debug_gyro_ema_y;
  data->debug.gyro_filter_z      = fusion_ctx.debug_gyro_ema_z;
  data->debug.compass_raw_x      = fusion_ctx.debug_compass_raw_x;
  data->debug.compass_raw_y      = fusion_ctx.debug_compass_raw_y;
  data->debug.compass_raw_z      = fusion_ctx.debug_compass_raw_z;
  data->debug.compass_filter_x   = fusion_ctx.compass_ema_x;
  data->debug.compass_filter_y   = fusion_ctx.compass_ema_y;
  data->debug.compass_filter_z   = fusion_ctx.compass_ema_z;
  data->debug.v_ins              = fusion_ctx.velocity_ins;
  data->debug.v_gps              = fusion_ctx.velocity_gps;
  data->debug.v_out              = fusion_ctx.velocity_out;
  data->debug.distance_ins       = fusion_ctx.distance_ins;
  data->debug.distance_gps       = fusion_ctx.debug_distance_gps;
  data->debug.distance_ins_total = fusion_ctx.distance_ins_total;
  data->debug.distance_gps_total = fusion_ctx.distance_gps_total;
  data->debug.acc_forward        = fusion_ctx.acc_forward;
  data->debug.roll_deg           = fusion_ctx.roll_rad * (180.0f / (float) M_PI);
  data->debug.pitch_deg          = fusion_ctx.pitch_rad * (180.0f / (float) M_PI);
  data->debug.gps_state          = (uint8_t) fusion_ctx.gps_state;
  data->debug.is_stationary      = fusion_ctx.is_stationary ? 1 : 0;
  data->debug.gps_reliable       = fusion_ctx.gps_reliable ? 1 : 0;
  data->debug.satellites         = (uint8_t) fusion_ctx.gps_data_buffer.satellites;
  data->debug.hdop               = fusion_ctx.gps_data_buffer.hdop;
  data->debug.lat                = fusion_ctx.gps_data_buffer.latitude;
  data->debug.lon                = fusion_ctx.gps_data_buffer.longitude;
#endif

  fusion_ctx.last_update_us            = current_time_us;
  fusion_ctx.is_new_gps_data_available = false;

#if (DEVICE_FUSION_DEBUG_LOG_ENABLED == 1)
  static size_t last_log_ms = 0;
  if (current_time_ms - last_log_ms >= 100)  // 10 Hz logging
  {
    last_log_ms = current_time_ms;
    sys_fusion_log_push(data, current_time_ms);
  }
#endif

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
#if (DEVICE_FUSION_ACC_FILTER == DEVICE_FUSION_FILTER_BTW) || (DEVICE_FUSION_COMPASS_FILTER == DEVICE_FUSION_FILTER_BTW)
static void biquad_reset(biquad_state_t *s, float v)
{
  s->x1 = s->x2 = s->y1 = s->y2 = v;
}

static float biquad_process(biquad_state_t *s, const biquad_coefs_t *c, float x)
{
  float y = c->b0 * x + c->b1 * s->x1 + c->b2 * s->x2 - c->a1 * s->y1 - c->a2 * s->y2;
  s->x2   = s->x1;
  s->x1   = x;
  s->y2   = s->y1;
  s->y1   = y;
  return y;
}

#if (DEVICE_FUSION_ACC_FILTER == DEVICE_FUSION_FILTER_BTW)
static const biquad_coefs_t k_acc_bw = { BW_B0, BW_B1, BW_B2, BW_A1, BW_A2 };
#endif

#if (DEVICE_FUSION_COMPASS_FILTER == DEVICE_FUSION_FILTER_BTW)
static const biquad_coefs_t k_compass_bw = { COMPASS_BW_B0, COMPASS_BW_B1, COMPASS_BW_B2, COMPASS_BW_A1,
                                             COMPASS_BW_A2 };
#endif
#endif

static void sys_fusion_calculate_offset_mag(void)
{
  float    sum   = 0.0f;
  float    sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  uint16_t valid = 0;
  for (uint16_t i = 0; i < ACC_OFFSET_MAGNITUDE_SAMPLE; i++)
  {
    bsp_acc_raw_data_t d = { 0 };
    if (bsp_acc_get_raw_data(&d) == STATUS_OK)
    {
      sum += sys_fusion_calculate_magnitude((float) d.acc_x, (float) d.acc_y, (float) d.acc_z);
      sum_x += d.acc_x;
      sum_y += d.acc_y;
      sum_z += d.acc_z;
      valid++;
    }
    delay(5);  // CPU busy waiting
  }

  fusion_ctx.offset_magnitude    = sum / (float) ACC_OFFSET_MAGNITUDE_SAMPLE;
  fusion_ctx.is_offset_mag_ready = true;
  LOG_DBG("Offset calibrated: %.4f g", fusion_ctx.offset_magnitude);

  // Seed initial tilt + acc EMA from the averaged stationary samples
  if (valid > 0)
  {
    float avg_x = sum_x / (float) valid;
    float avg_y = sum_y / (float) valid;
    float avg_z = sum_z / (float) valid;

    fusion_ctx.roll_rad  = atan2f(avg_y, avg_z);
    fusion_ctx.pitch_rad = atan2f(-avg_x, hypotf(avg_y, avg_z));

    fusion_ctx.acc_ema_x    = avg_x;
    fusion_ctx.acc_ema_y    = avg_y;
    fusion_ctx.acc_ema_z    = avg_z;
    fusion_ctx.acc_ema_init = true;

    LOG_DBG("Initial tilt: roll=%.2f pitch=%.2f deg", fusion_ctx.roll_rad * 180.0f / (float) M_PI,
            fusion_ctx.pitch_rad * 180.0f / (float) M_PI);
  }
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

static float sys_fusion_wrap_to_180(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static bool sys_fusion_read_imu(bsp_acc_raw_data_t *imu)
{
  if (bsp_acc_get_raw_data(imu) != STATUS_OK)
    return false;

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.debug_acc_raw_x  = imu->acc_x;
  fusion_ctx.debug_acc_raw_y  = imu->acc_y;
  fusion_ctx.debug_acc_raw_z  = imu->acc_z;
  fusion_ctx.debug_gyro_raw_x = imu->gyro_x;
  fusion_ctx.debug_gyro_raw_y = imu->gyro_y;
  fusion_ctx.debug_gyro_raw_z = imu->gyro_z;

  if (!fusion_ctx.debug_gyro_ema_init)
  {
    fusion_ctx.debug_gyro_ema_x    = imu->gyro_x;
    fusion_ctx.debug_gyro_ema_y    = imu->gyro_y;
    fusion_ctx.debug_gyro_ema_z    = imu->gyro_z;
    fusion_ctx.debug_gyro_ema_init = true;
  }
  else
  {
    fusion_ctx.debug_gyro_ema_x =
      DBG_GYRO_EMA_ALPHA * imu->gyro_x + (1.0f - DBG_GYRO_EMA_ALPHA) * fusion_ctx.debug_gyro_ema_x;
    fusion_ctx.debug_gyro_ema_y =
      DBG_GYRO_EMA_ALPHA * imu->gyro_y + (1.0f - DBG_GYRO_EMA_ALPHA) * fusion_ctx.debug_gyro_ema_y;
    fusion_ctx.debug_gyro_ema_z =
      DBG_GYRO_EMA_ALPHA * imu->gyro_z + (1.0f - DBG_GYRO_EMA_ALPHA) * fusion_ctx.debug_gyro_ema_z;
  }
#endif

  // Acc filter — per-axis body frame (EMA or Butterworth, see DEVICE_FUSION_ACC_FILTER)
#if (DEVICE_FUSION_ACC_FILTER == DEVICE_FUSION_FILTER_BTW)
  if (!fusion_ctx.bw_init)
  {
    biquad_reset(&fusion_ctx.bw_x, imu->acc_x);
    biquad_reset(&fusion_ctx.bw_y, imu->acc_y);
    biquad_reset(&fusion_ctx.bw_z, imu->acc_z);
    fusion_ctx.bw_init = true;
  }
  fusion_ctx.acc_ema_x = biquad_process(&fusion_ctx.bw_x, &k_acc_bw, imu->acc_x);
  fusion_ctx.acc_ema_y = biquad_process(&fusion_ctx.bw_y, &k_acc_bw, imu->acc_y);
  fusion_ctx.acc_ema_z = biquad_process(&fusion_ctx.bw_z, &k_acc_bw, imu->acc_z);
#else
  if (!fusion_ctx.acc_ema_init)
  {
    fusion_ctx.acc_ema_x    = imu->acc_x;
    fusion_ctx.acc_ema_y    = imu->acc_y;
    fusion_ctx.acc_ema_z    = imu->acc_z;
    fusion_ctx.acc_ema_init = true;
  }
  else
  {
    fusion_ctx.acc_ema_x = ACC_EMA_ALPHA * imu->acc_x + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.acc_ema_x;
    fusion_ctx.acc_ema_y = ACC_EMA_ALPHA * imu->acc_y + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.acc_ema_y;
    fusion_ctx.acc_ema_z = ACC_EMA_ALPHA * imu->acc_z + (1.0f - ACC_EMA_ALPHA) * fusion_ctx.acc_ema_z;
  }
#endif
  return true;
}

static void sys_fusion_update_attitude(const bsp_acc_raw_data_t *imu, bool compass_fresh, float dt)
{
  // Roll/pitch complementary filter (gyro predict + accel correct)
  float roll_acc  = atan2f(imu->acc_y, imu->acc_z);
  float pitch_acc = atan2f(-imu->acc_x, hypotf(imu->acc_y, imu->acc_z));

  float gyro_x_rads = (imu->gyro_x * DEG_TO_RAD) - fusion_ctx.gyro_bias_x;
  float gyro_y_rads = (imu->gyro_y * DEG_TO_RAD) - fusion_ctx.gyro_bias_y;

  fusion_ctx.roll_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.roll_rad + gyro_x_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * roll_acc;
  fusion_ctx.pitch_rad =
    ATTITUDE_GYRO_WEIGHT * (fusion_ctx.pitch_rad + gyro_y_rads * dt) + (1.0f - ATTITUDE_GYRO_WEIGHT) * pitch_acc;

  // Yaw predict — high-rate gyro_z integration
  float gyro_z_rads            = (imu->gyro_z * DEG_TO_RAD) - fusion_ctx.gyro_bias_z;
  fusion_ctx.latest_gyro_z_dps = fabsf(gyro_z_rads) * (180.0f / (float) M_PI);
#if (DEVICE_FUSION_YAW_CF_ENABLED)
  if (fusion_ctx.yaw_init)
  {
    fusion_ctx.heading_deg += gyro_z_rads * dt * (180.0f / (float) M_PI);
    if (fusion_ctx.heading_deg >= 360.0f)
      fusion_ctx.heading_deg -= 360.0f;
    else if (fusion_ctx.heading_deg < 0.0f)
      fusion_ctx.heading_deg += 360.0f;
  }
#endif

  if (fusion_ctx.is_stationary)
  {
    fusion_ctx.gyro_bias_x += GYRO_BIAS_ALPHA * ((imu->gyro_x * DEG_TO_RAD) - fusion_ctx.gyro_bias_x);
    fusion_ctx.gyro_bias_y += GYRO_BIAS_ALPHA * ((imu->gyro_y * DEG_TO_RAD) - fusion_ctx.gyro_bias_y);
    fusion_ctx.gyro_bias_z += GYRO_BIAS_ALPHA * ((imu->gyro_z * DEG_TO_RAD) - fusion_ctx.gyro_bias_z);
  }

  // Yaw correct — low-rate compass, tilt-compensated with this cycle's roll/pitch
  if (compass_fresh)
  {
    float mag_x = fusion_ctx.compass_ema_x;
    float mag_y = fusion_ctx.compass_ema_y;
    float mag_z = fusion_ctx.compass_ema_z;

    float cos_roll  = cosf(fusion_ctx.roll_rad);
    float sin_roll  = sinf(fusion_ctx.roll_rad);
    float cos_pitch = cosf(fusion_ctx.pitch_rad);
    float sin_pitch = sinf(fusion_ctx.pitch_rad);

    float Xh = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z * cos_roll * sin_pitch;
    float Yh = mag_y * cos_roll - mag_z * sin_roll;

    float heading_deg = atan2f(Yh, Xh) * 180.0f / (float) M_PI;
    if (heading_deg < 0.0f)
      heading_deg += 360.0f;

    heading_deg -= YAW_OFFSET_DEG;
    if (heading_deg >= 360.0f)
      heading_deg -= 360.0f;
    else if (heading_deg < 0.0f)
      heading_deg += 360.0f;

    if (!fusion_ctx.yaw_init)
    {
      fusion_ctx.heading_deg = heading_deg;
      fusion_ctx.yaw_init    = true;
    }
    else
    {
#if (DEVICE_FUSION_YAW_CF_ENABLED)
      float err = sys_fusion_wrap_to_180(heading_deg - fusion_ctx.heading_deg);
      float weight =
        (fusion_ctx.latest_gyro_z_dps > YAW_ROTATING_TH_DEGPS) ? YAW_GYRO_WEIGHT : YAW_GYRO_WEIGHT_STATIONARY;
      fusion_ctx.heading_deg += (1.0f - weight) * err;
      if (fusion_ctx.heading_deg >= 360.0f)
        fusion_ctx.heading_deg -= 360.0f;
      else if (fusion_ctx.heading_deg < 0.0f)
        fusion_ctx.heading_deg += 360.0f;
#else
      fusion_ctx.heading_deg = heading_deg;  // direct compass heading
#endif
    }
  }

  fusion_ctx.direction_str = sys_fusion_deg_to_direction_str(fusion_ctx.heading_deg);
}

static void sys_fusion_update_ins_velocity(float dt)
{
  float acc_x = fusion_ctx.acc_ema_x;
  float acc_y = fusion_ctx.acc_ema_y;
  float acc_z = fusion_ctx.acc_ema_z;

  // Body frame -> Navigation frame rotation (ZYX Euler, yaw from compass)
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

  // Forward projection — per-axis Butterworth already cleans the input
  float acc_forward_raw = acc_north * cos_yaw + acc_east * sin_yaw;

  float mag_g        = hypotf(hypotf(acc_x, acc_y), acc_z);
  fusion_ctx.acc_raw = mag_g - fusion_ctx.offset_magnitude;

  fusion_ctx.acc_forward = acc_forward_raw;

  // INS velocity integration
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

  // Accumulate INS distance
  if (fusion_ctx.velocity_ins > GPS_SPEED_MIN_MS)
  {
    fusion_ctx.distance_ins += fusion_ctx.velocity_ins * dt;
    fusion_ctx.distance_ins_total += fusion_ctx.velocity_ins * dt;
  }
}

static void sys_fusion_update_gps_data(void)
{
  if (!fusion_ctx.gps_ready)
    return;

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
    fusion_ctx.velocity_gps = raw_speed;
    float anchor            = GPS_ANCHOR_RATE;
    if (fabsf(fusion_ctx.acc_forward) > ACTIVE_MOTION_TH_MS2)
      anchor *= GPS_ANCHOR_TRANSIENT_SCALE;

    fusion_ctx.velocity_ins              = (1.0f - anchor) * fusion_ctx.velocity_ins + anchor * fusion_ctx.velocity_gps;
    fusion_ctx.is_new_gps_fix_this_cycle = true;  // FIX: mark fresh GPS fix for CF
  }

  fusion_ctx.last_gps_ms = OS_GET_TICK();

  if (is_gps_data_ok)
  {
    float lat             = (float) fusion_ctx.gps_data_buffer.latitude;
    float lon             = (float) fusion_ctx.gps_data_buffer.longitude;
    bool  accept_position = false;

    if (fusion_ctx.has_last_gps_position)
    {
      float distance_gps = sys_fusion_haversine_m(fusion_ctx.last_valid_lat, fusion_ctx.last_valid_lon, lat, lon);

#if (DEVICE_FUSION_DEBUG_MODE == 1)
      fusion_ctx.debug_distance_gps = distance_gps;
#endif
      if (distance_gps < GPS_MAX_STEP_M)
        fusion_ctx.distance_gps_total += distance_gps;

      // GPS reliability check (Chiang 2013):
      // z_r = |d_INS - d_GPS|; reject GPS if residual exceeds threshold
      float z_r = fabsf(fusion_ctx.distance_ins - distance_gps);
      if (z_r < GPS_RELIABILITY_THRESHOLD_M)
      {
        fusion_ctx.gps_reliable = true;
        accept_position         = true;
        if (distance_gps < GPS_MAX_STEP_M && fusion_ctx.velocity_gps > GPS_SPEED_MIN_MS)
        {
          float inc = distance_gps - fusion_ctx.distance_gap_ins;
          if (inc > 0.0f)
            fusion_ctx.distance_m += inc;
        }
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
      accept_position         = true;
    }

    if (accept_position)
    {
      fusion_ctx.last_valid_lat        = lat;
      fusion_ctx.last_valid_lon        = lon;
      fusion_ctx.has_last_gps_position = true;
      fusion_ctx.distance_ins          = 0.0f;
      fusion_ctx.distance_gap_ins      = 0.0f;
    }
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

static bool sys_fusion_detect_zupt(float accel_ms2, float dt)
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
      // Snap all velocities to zero — seeds CF cleanly for the next motion onset
      fusion_ctx.velocity_ins = 0.0f;
      fusion_ctx.velocity_gps = 0.0f;
      fusion_ctx.velocity_out = 0.0f;
      return true;
    }
  }
  else
  {
    fusion_ctx.is_stationary      = false;
    fusion_ctx.stationary_time_ms = 0;
  }
  return false;
}

static void sys_fusion_compute_output_velocity(sys_fusion_data_t *data, float dt)
{
  // GPS branch: only inject when a fresh fix arrived this cycle
  bool use_gps =
    fusion_ctx.is_new_gps_fix_this_cycle && (fusion_ctx.gps_state == GPS_STATE_ACTIVE) && fusion_ctx.gps_reliable;
  float v_gps_eff = use_gps ? fusion_ctx.velocity_gps : 0.0f;
  float v_ref     = fmaxf(fusion_ctx.velocity_ins, v_gps_eff);

  bool near_stop    = (v_ref < VEL_NEAR_ZERO_MS);
  bool no_drive_acc = (fusion_ctx.acc_forward < ACC_THRESHOLD_MS2);
  bool hard_motion  = (fabsf(fusion_ctx.acc_forward) > CF_WC_TRANSIENT_TH_MS2);
  bool gps_agrees   = (fusion_ctx.gps_state == GPS_STATE_ACTIVE)
                    && (fabsf(fusion_ctx.velocity_ins - fusion_ctx.velocity_gps) < CF_TRANSIENT_AGREE_MS);
  float cf_wc_effect;
  if (near_stop && no_drive_acc)
    cf_wc_effect = CF_WC_STOPPING;
  else if (hard_motion && gps_agrees)
    cf_wc_effect = CF_WC_TRANSIENT;
  else
    cf_wc_effect = CF_WC;

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

  float denom_ins = 1.0f + cf_wc_effect * dt;
  float gamma     = 1.0f / denom_ins;
  float alpha     = (cf_wc_effect * dt) / denom_ins;
  float beta      = use_gps ? (cf_wc_effect * dt_gps / (1.0f + cf_wc_effect * dt_gps)) : 0.0f;
  float gamma_adj = use_gps ? (1.0f - alpha - beta) : gamma;

  if (gamma_adj < 0.0f)
    gamma_adj = 0.0f;

  fusion_ctx.velocity_out = gamma_adj * fusion_ctx.velocity_out + alpha * fusion_ctx.velocity_ins + beta * v_gps_eff;

  if (fusion_ctx.velocity_out < 0.0f)
    fusion_ctx.velocity_out = 0.0f;
  else if (v_ref < VEL_SETTLE_BAND_MS && fusion_ctx.velocity_out < VEL_SETTLE_BAND_MS)
    fusion_ctx.velocity_out = 0.0f;

  // Vout safety anchor — bounds output drift when vins is suspect
#if (VOUT_ANCHOR_MODE == VOUT_ANCHOR_SOFT)
  if (fusion_ctx.gps_state == GPS_STATE_ACTIVE && fusion_ctx.velocity_gps > GPS_SPEED_MIN_MS)
  {
    fusion_ctx.velocity_out =
      (1.0f - VOUT_ANCHOR_SOFT_RATE) * fusion_ctx.velocity_out + VOUT_ANCHOR_SOFT_RATE * fusion_ctx.velocity_gps;
  }
#elif (VOUT_ANCHOR_MODE == VOUT_ANCHOR_SNAP)
  if (fusion_ctx.gps_state == GPS_STATE_ACTIVE && fusion_ctx.velocity_gps > GPS_SPEED_MIN_MS
      && fabsf(fusion_ctx.velocity_out - fusion_ctx.velocity_gps) > VOUT_SNAP_TH_MS)
  {
    fusion_ctx.velocity_out = fusion_ctx.velocity_gps;
    fusion_ctx.velocity_ins = fusion_ctx.velocity_gps;
  }
#endif

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

static bool sys_fusion_preprocess_data(size_t current_ms)
{
  if (!fusion_ctx.compass_ready)
    return false;
  if ((current_ms - fusion_ctx.compass_last_ms) < COMPASS_UPDATE_MS)
    return false;
  fusion_ctx.compass_last_ms = current_ms;

  bsp_compass_raw_data_t raw_data;
  if (bsp_compass_read_raw(&raw_data) != STATUS_OK)
  {
    LOG_ERR("Read compass fail");
    return false;
  }

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  fusion_ctx.debug_compass_raw_x = (float) raw_data.raw_x;
  fusion_ctx.debug_compass_raw_y = (float) raw_data.raw_y;
  fusion_ctx.debug_compass_raw_z = (float) raw_data.raw_z;
#endif

#if (DEVICE_FUSION_COMPASS_FILTER == DEVICE_FUSION_FILTER_BTW)
  if (!fusion_ctx.compass_filter_init)
  {
    biquad_reset(&fusion_ctx.compass_bw_x, (float) raw_data.raw_x);
    biquad_reset(&fusion_ctx.compass_bw_y, (float) raw_data.raw_y);
    biquad_reset(&fusion_ctx.compass_bw_z, (float) raw_data.raw_z);
    fusion_ctx.compass_filter_init = true;
  }
  fusion_ctx.compass_ema_x = biquad_process(&fusion_ctx.compass_bw_x, &k_compass_bw, (float) raw_data.raw_x);
  fusion_ctx.compass_ema_y = biquad_process(&fusion_ctx.compass_bw_y, &k_compass_bw, (float) raw_data.raw_y);
  fusion_ctx.compass_ema_z = biquad_process(&fusion_ctx.compass_bw_z, &k_compass_bw, (float) raw_data.raw_z);
#else
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
#endif
  return true;
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

#endif  // DEVICE_FUSION_ALGO == DEVICE_FUSION_ALGO_V2

/* End of file -------------------------------------------------------- */