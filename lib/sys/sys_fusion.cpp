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
LOG_MODULE_REGISTER(sys_fusion, LOG_LEVEL_DBG)

// #define DEMO_VEHICLE (1)
#define DEMO_WALKING                (1)

// Acc parameters
#define ACC_FILTER_ALPHA            (0.3f)
#define ACC_THRESHOLD               (0.05f)
#define ACC_OFFSET_MAGNITUDE_SAMPLE (200)

#if defined(DEMO_VEHICLE)
#define ZUPT_ACC_THRESHOLD     (0.03f)
#define ZUPT_TIME_THRESHOLD_MS (1000)
#define INS_DECAY_NORMAL       (0.9998f)  // Very slow decay while riding
#define INS_DECAY_STOPPING     (0.94f)    // Fast decay when stopped
#define INS_DECAY_GPS_LOST     (0.97f)    // Medium decay during GPS fade-out
#define GPS_SPEED_MIN_MS       (0.6f)     // 2.2 km/h
#define GPS_ANCHOR_RATE        (0.7f)     // Strong GPS anchor outdoors

#elif defined(DEMO_WALKING)
#define ZUPT_ACC_THRESHOLD     (0.015f)
#define ZUPT_TIME_THRESHOLD_MS (1500)
#define INS_DECAY_NORMAL       (0.9995f)  // Slow decay while walking
#define INS_DECAY_STOPPING     (0.92f)    // Fast decay ~0.5s to zero
#define INS_DECAY_GPS_LOST     (0.96f)    // Medium decay when GPS fading out
#define GPS_SPEED_MIN_MS       (0.4f)     // 1.4 km/h
#define GPS_ANCHOR_RATE        (0.7f)     // Stronger anchor = more responsive

#else
#error "Must define either DEMO_VEHICLE or DEMO_WALKING"
#endif

#define GPS_HDOP_MAX         (3.0f)
#define GPS_SATELLITES_MIN   (4)
#define GPS_EMA_ALPHA        (0.6f)  // Higher = faster GPS response to speed changes
#define GPS_MAX_STEP_M       (50.0f)
#define GPS_VALID_TIMEOUT_MS (2000)  // Reduced: less stale GPS before fade-out
#define GPS_FADE_TIMEOUT_MS  (1000)  // How long to fade INS after GPS lost

#define COMPASS_EMA_ALPHA    (0.15f)
#define COMPASS_UPDATE_MS    (100)

#define GRAVITY_MS2          (9.806f)
#define KMH_TO_MS            (1.0f / 3.6f)
#define MS_TO_KMH            (3.6f)
#define US_TO_S              (1000000.0f)

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

  // INS
  size_t last_update_us;
  float  velocity_ins;
  float  acc_filtered;
  float  acc_raw;
  float  offset_magnitude;
  float  distance_m;

  // ZUPT
  bool     is_stationary;
  uint32_t stationary_time_ms;

  // Compass
  float       compass_ema_x;
  float       compass_ema_y;
  float       compass_ema_z;
  float       compass_ema_alpha;
  size_t      compass_last_ms;
  bool        compass_filter_init;
  float       heading_deg;
  const char *direction_str;

  // Sensor flags
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
static float             sys_fusion_calculate_magnitude(float x, float y, float z);
static status_function_t sys_fusion_calculate_offset_mag(void);
static void              sys_fusion_update_ins_velocity(float dt);
static void              sys_fusion_update_gps_data(void);
static void              sys_fusion_update_gps_state(size_t current_ms);
static void              sys_fusion_detect_zupt(float accel_ms2, float dt);
static void              sys_fusion_compute_output_velocity(sys_fusion_data_t *data);
static void              sys_fusion_read_compass(sys_fusion_data_t *data, size_t current_ms);
static const char       *sys_fusion_deg_to_direction_str(float deg);
static void              sys_fusion_gps_callback(bsp_gps_data_t *gps_data);
static float             sys_fusion_haversine_m(float lat1, float lon1, float lat2, float lon2);

/* Function definitions ----------------------------------------------- */

void sys_fusion_init(void)
{
  if (fusion_ctx.initialized)
    return;

  fusion_ctx.last_gps_ms           = 0;
  fusion_ctx.gps_lost_ms           = 0;
  fusion_ctx.gps_state             = GPS_STATE_INVALID;
  fusion_ctx.velocity_ins          = 0.0f;
  fusion_ctx.velocity_gps          = 0.0f;
  fusion_ctx.acc_filtered          = 0.0f;
  fusion_ctx.acc_raw               = 0.0f;
  fusion_ctx.offset_magnitude      = 0.0f;
  fusion_ctx.distance_m            = 0.0f;
  fusion_ctx.has_last_gps_position = false;
  fusion_ctx.last_valid_lat        = 0.0f;
  fusion_ctx.last_valid_lon        = 0.0f;
  fusion_ctx.last_update_us        = 0;

  fusion_ctx.is_stationary      = false;
  fusion_ctx.stationary_time_ms = 0;

  fusion_ctx.acc_ready     = false;
  fusion_ctx.gps_ready     = false;
  fusion_ctx.compass_ready = false;

  fusion_ctx.compass_ema_x       = 0.0f;
  fusion_ctx.compass_ema_y       = 0.0f;
  fusion_ctx.compass_ema_z       = 0.0f;
  fusion_ctx.compass_ema_alpha   = COMPASS_EMA_ALPHA;
  fusion_ctx.compass_last_ms     = 0;
  fusion_ctx.compass_filter_init = false;
  fusion_ctx.heading_deg         = 0.0f;
  fusion_ctx.direction_str       = "N";

  fusion_ctx.is_offset_mag_ready = false;
  fusion_ctx.initialized         = true;

  LOG_DBG("Init ACC");
  if (bsp_acc_init() == STATUS_OK)
  {
    fusion_ctx.acc_ready = true;
    LOG_DBG("ACC OK");
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

  LOG_DBG("Calibrating — keep device stationary...");
  if (fusion_ctx.acc_ready)
  {
    (void) sys_fusion_calculate_offset_mag();
  }
}

status_function_t sys_fusion_process(sys_fusion_data_t *data)
{
  if (data == NULL || !fusion_ctx.initialized)
    return STATUS_ERROR;

  size_t current_time_us = micros();
  size_t current_time_ms = OS_GET_TICK();
  float  dt = (fusion_ctx.last_update_us == 0) ? 0.02f : (current_time_us - fusion_ctx.last_update_us) / US_TO_S;
  if (dt > 0.1f)
    dt = 0.1f;

  // 1. GPS
  sys_fusion_update_gps_data();
  sys_fusion_update_gps_state(current_time_ms);

  // 2. INS
  if (fusion_ctx.is_offset_mag_ready && fusion_ctx.acc_ready)
  {
    sys_fusion_update_ins_velocity(dt);
  }

  // 3. ZUPT
  sys_fusion_detect_zupt(fusion_ctx.acc_raw * GRAVITY_MS2, dt);

  // 4. Output velocity
  sys_fusion_compute_output_velocity(data);

  // 5. INS-only distance fallback when GPS unavailable
  if (fusion_ctx.gps_state == GPS_STATE_INVALID && fusion_ctx.is_offset_mag_ready && dt > 0.0f
      && data->velocity_ms > GPS_SPEED_MIN_MS)
  {
    fusion_ctx.distance_m += data->velocity_ms * dt;
  }

  // 6. Compass
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
static status_function_t sys_fusion_calculate_offset_mag(void)
{
  if (!fusion_ctx.initialized)
    return STATUS_ERROR;

  float sum = 0.0f;
  for (uint16_t i = 0; i < ACC_OFFSET_MAGNITUDE_SAMPLE; i++)
  {
    bsp_acc_raw_data_t d = { 0 };
    if (bsp_acc_get_raw_data(&d) == STATUS_OK)
    {
      sum += sys_fusion_calculate_magnitude((float) d.acc_x, (float) d.acc_y, (float) d.acc_z);
    }
    delay(5);
  }

  fusion_ctx.offset_magnitude    = sum / (float) ACC_OFFSET_MAGNITUDE_SAMPLE;
  fusion_ctx.is_offset_mag_ready = true;
  LOG_DBG("Offset calibrated: %.4f", fusion_ctx.offset_magnitude);
  return STATUS_OK;
}

static float sys_fusion_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

static void sys_fusion_update_ins_velocity(float dt)
{
  bsp_acc_raw_data_t accel_data = { 0 };
  if (bsp_acc_get_raw_data(&accel_data) != STATUS_OK)
    return;

  float mag =
    sys_fusion_calculate_magnitude((float) accel_data.acc_x, (float) accel_data.acc_y, (float) accel_data.acc_z);
  float acc_clear = mag - fusion_ctx.offset_magnitude;

  fusion_ctx.acc_raw = acc_clear;  // Save raw for ZUPT

  fusion_ctx.acc_filtered = ACC_FILTER_ALPHA * acc_clear + (1.0f - ACC_FILTER_ALPHA) * fusion_ctx.acc_filtered;

  float accel_ms2 = fusion_ctx.acc_filtered * GRAVITY_MS2;

  if (fabsf(accel_ms2) > ACC_THRESHOLD)
  {
    fusion_ctx.velocity_ins += accel_ms2 * dt;
  }
  else
  {
    // Select decay rate based on state — stationary always takes priority
    float decay;
    if (fusion_ctx.is_stationary)
    {
      decay = INS_DECAY_STOPPING;  // Fast: ~0.5s to near-zero
    }
    else if (fusion_ctx.gps_state == GPS_STATE_FADING)
    {
      decay = INS_DECAY_GPS_LOST;  // Medium: smooth drop during GPS fade-out
    }
    else
    {
      decay = INS_DECAY_NORMAL;  // Slow: preserve velocity while moving
    }

    fusion_ctx.velocity_ins *= powf(decay, dt / 0.02f);
  }

  if (fusion_ctx.velocity_ins < 0.0f)
    fusion_ctx.velocity_ins = 0.0f;
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

  float raw_speed = fusion_ctx.gps_data_buffer.speed_kmph * KMH_TO_MS;
  bool  is_gps_data_ok =
    (fusion_ctx.gps_data_buffer.hdop < GPS_HDOP_MAX) && (fusion_ctx.gps_data_buffer.satellites >= GPS_SATELLITES_MIN);

  if (!is_gps_data_ok || raw_speed < GPS_SPEED_MIN_MS)
  {
    fusion_ctx.velocity_gps = 0.0f;
    fusion_ctx.velocity_ins *= (1.0f - GPS_ANCHOR_RATE);
  }
  else
  {
    // Moving with good GPS — EMA smooth + re-anchor INS
    fusion_ctx.velocity_gps = GPS_EMA_ALPHA * raw_speed + (1.0f - GPS_EMA_ALPHA) * fusion_ctx.velocity_gps;

    // Re-anchor INS: GPS_ANCHOR_RATE=0.7 means INS is pulled 70% toward GPS each update
    fusion_ctx.velocity_ins =
      (1.0f - GPS_ANCHOR_RATE) * fusion_ctx.velocity_ins + GPS_ANCHOR_RATE * fusion_ctx.velocity_gps;
  }

  fusion_ctx.last_gps_ms = OS_GET_TICK();

  if (is_gps_data_ok)
  {
    float lat = fusion_ctx.gps_data_buffer.latitude;
    float lon = fusion_ctx.gps_data_buffer.longitude;

    if (fusion_ctx.has_last_gps_position)
    {
      float d = sys_fusion_haversine_m(fusion_ctx.last_valid_lat, fusion_ctx.last_valid_lon, lat, lon);
      if (d < GPS_MAX_STEP_M && fusion_ctx.velocity_gps > GPS_SPEED_MIN_MS)
      {
        fusion_ctx.distance_m += d;
      }
    }

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
      LOG_DBG("GPS: INVALID → ACTIVE");
    }
    break;

  case GPS_STATE_ACTIVE:
    if (!gps_recently_updated)
    {
      fusion_ctx.gps_state   = GPS_STATE_FADING;
      fusion_ctx.gps_lost_ms = current_ms;
      LOG_DBG("GPS: ACTIVE → FADING");
    }
    break;

  case GPS_STATE_FADING:
    if (gps_recently_updated && fusion_ctx.velocity_gps > 0.0f)
    {
      fusion_ctx.gps_state = GPS_STATE_ACTIVE;
      LOG_DBG("GPS: FADING → ACTIVE");
    }
    else if ((current_ms - fusion_ctx.gps_lost_ms) >= GPS_FADE_TIMEOUT_MS)
    {
      fusion_ctx.gps_state    = GPS_STATE_INVALID;
      fusion_ctx.velocity_gps = 0.0f;
      LOG_DBG("GPS: FADING → INVALID");
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

static void sys_fusion_compute_output_velocity(sys_fusion_data_t *data)
{
  float output = fusion_ctx.velocity_ins;
  if (output < 0.0f)
    output = 0.0f;

  data->velocity_ms  = output;
  data->velocity_kmh = output * MS_TO_KMH;
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
  // Always return the latest known heading, even if we skip or fail this cycle.
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

  float alpha = fusion_ctx.compass_ema_alpha;
  if (!fusion_ctx.compass_filter_init)
  {
    fusion_ctx.compass_ema_x       = (float) raw_data.raw_x;
    fusion_ctx.compass_ema_y       = (float) raw_data.raw_y;
    fusion_ctx.compass_ema_z       = (float) raw_data.raw_z;
    fusion_ctx.compass_filter_init = true;
  }
  else
  {
    fusion_ctx.compass_ema_x = alpha * (float) raw_data.raw_x + (1.0f - alpha) * fusion_ctx.compass_ema_x;
    fusion_ctx.compass_ema_y = alpha * (float) raw_data.raw_y + (1.0f - alpha) * fusion_ctx.compass_ema_y;
    fusion_ctx.compass_ema_z = alpha * (float) raw_data.raw_z + (1.0f - alpha) * fusion_ctx.compass_ema_z;
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
