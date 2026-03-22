/**
 * @file       sys_input.cpp
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      System Input Layer - Sensor data management implementation
 *
 * @details    Manages input from sensors and calculates velocity and distance
 *
 * @changelog
 *   - Fix low sensitivity outdoors: GPS_ANCHOR_RATE increased, GPS_EMA_ALPHA increased
 *   - Fix GPS→indoor transition jerk:
 *       (1) GPS_VALID_TIMEOUT_MS reduced to 2000ms (less stale GPS lingers)
 *       (2) Smooth GPS fade-out: when GPS goes stale, INS_DECAY_STOPPING kicks in
 *           immediately instead of hard-zeroing velocity_gps after timeout
 *       (3) GPS transition state: is_gps_fading tracks the fade-out period
 *   - Fix jerky velocity: INS interpolates between GPS updates at 50Hz
 *   - Fix slow stop: GPS zeroed immediately + INS_DECAY_STOPPING on stationary
 *   - Fix ZUPT bypass: velocity_ins NOT overwritten by fused result
 *   - Fix distance drift: Haversine only when velocity_gps > threshold
 */

/* Includes ----------------------------------------------------------- */
#include "sys_input.h"

#include "bsp_acc.h"
#include "bsp_batt.h"
#include "bsp_compass.h"
#include "bsp_dust_sensor.h"
#include "bsp_gps.h"
#include "bsp_io.h"
#include "bsp_temp_hum.h"
#include "log_service.h"
#include "os_lib.h"

#include <math.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_input, LOG_LEVEL_DBG)

// Select ONE mode only
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

#define GPS_HDOP_MAX                   (3.0f)
#define GPS_SATELLITES_MIN             (4)
#define GPS_EMA_ALPHA                  (0.6f)  // Higher = faster GPS response to speed changes
#define GPS_MAX_STEP_M                 (50.0f)
#define GPS_VALID_TIMEOUT_MS           (2000)  // Reduced: less stale GPS before fade-out
#define GPS_FADE_TIMEOUT_MS            (1000)  // How long to fade INS after GPS lost

#define COMPASS_EMA_ALPHA              (0.15f)
#define COMPASS_UPDATE_MS              (100)

#define GRAVITY_MS2                    (9.806f)
#define KMH_TO_MS                      (1.0f / 3.6f)
#define MS_TO_KMH                      (3.6f)
#define US_TO_S                        (1000000.0f)

#define SYS_INPUT_DUST_EMA_ALPHA       (0.2f)

#define SYS_INPUT_BATT_INITIAL_SAMPLES (200)
#define SYS_INPUT_BATT_EMA_ALPHA       (0.1f)
#define SYS_INPUT_BATT_READ_VOLT_TIMES (20)
#define SYS_INPUT_BATT_DEBOUNCE        (10)
#define SYS_INPUT_BATT_MAX_ERROR       (5)

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief GPS state machine for smooth transition handling
 */
typedef enum
{
  GPS_STATE_INVALID = 0,  // No GPS or poor quality
  GPS_STATE_ACTIVE,       // GPS valid and recent
  GPS_STATE_FADING,       // GPS just lost — smooth fade-out period
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
  size_t         gps_lost_ms;  // Timestamp when GPS became invalid

  // INS
  size_t last_update_us;
  size_t last_update_ms;
  float  velocity_ins;
  float  acc_filtered;
  float  acc_raw;
  float  offset_magnitude;

  // ZUPT
  bool     is_stationary;
  uint32_t stationary_time_ms;

  // Compass
  float  compass_ema_x;
  float  compass_ema_y;
  float  compass_ema_z;
  float  compass_ema_alpha;
  size_t compass_last_ms;
  bool   compass_filter_init;

  // Sensor flags
  bool compass_ready;
  bool acc_ready;
  bool gps_ready;
  bool dust_ready;
  bool temp_hum_ready;
  bool initialized;
  bool is_offset_mag_ready;

  uint32_t last_env_update_ms;

  int32_t  batt_remaining_mah;
  uint32_t batt_last_update_ms;

  sys_input_data_t data;
} sys_input_context_t;

/* Private variables -------------------------------------------------- */
static sys_input_context_t input_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static float             sys_input_calculate_magnitude(float x, float y, float z);
static status_function_t sys_input_calculate_offset_mag(void);
static void              sys_input_update_ins_velocity(float dt);
static void              sys_input_update_gps_data(void);
static void              sys_input_update_gps_state(size_t current_ms);
static void              sys_input_detect_zupt(float accel_ms2, float dt);
static void              sys_input_compute_output_velocity(void);
static void              sys_input_read_dust_sensor(void);
static void              sys_input_read_compass(size_t current_ms);
static const char       *sys_input_deg_to_direction_str(float deg);
static void              sys_input_gps_callback(bsp_gps_data_t *data);
static float             sys_input_haversine_m(float lat1, float lon1, float lat2, float lon2);
static void              sys_input_trigger_io_event(void);
static void              sys_input_initial_battery_level(void);
static void              sys_input_read_battery_level(float *battery_level);

/* Function definitions ----------------------------------------------- */

void sys_input_init(void)
{
  if (input_ctx.initialized)
    return;

  LOG_DBG("Initializing system input...");

  input_ctx.data.velocity_ms          = 0.0f;
  input_ctx.data.velocity_kmh         = 0.0f;
  input_ctx.data.distance_m           = 0.0f;
  input_ctx.data.dust_value           = 0.0f;
  input_ctx.data.temp_hum.temperature = 0.0f;
  input_ctx.data.temp_hum.humidity    = 0.0f;
  input_ctx.data.timestamp_ms         = 0;
  input_ctx.last_env_update_ms        = 0;

  input_ctx.last_update_ms        = 0;
  input_ctx.last_update_us        = 0;
  input_ctx.last_gps_ms           = 0;
  input_ctx.gps_lost_ms           = 0;
  input_ctx.gps_state             = GPS_STATE_INVALID;
  input_ctx.velocity_ins          = 0.0f;
  input_ctx.velocity_gps          = 0.0f;
  input_ctx.acc_filtered          = 0.0f;
  input_ctx.acc_raw               = 0.0f;
  input_ctx.offset_magnitude      = 0.0f;
  input_ctx.has_last_gps_position = false;
  input_ctx.last_valid_lat        = 0.0f;
  input_ctx.last_valid_lon        = 0.0f;
  input_ctx.data.battery_level    = 100.0f;

  input_ctx.is_stationary      = false;
  input_ctx.stationary_time_ms = 0;

  input_ctx.acc_ready      = false;
  input_ctx.gps_ready      = false;
  input_ctx.dust_ready     = false;
  input_ctx.compass_ready  = false;
  input_ctx.temp_hum_ready = false;

  input_ctx.data.heading_deg    = 0.0f;
  input_ctx.data.direction_str  = "N";
  input_ctx.compass_ema_x       = 0.0f;
  input_ctx.compass_ema_y       = 0.0f;
  input_ctx.compass_ema_z       = 0.0f;
  input_ctx.compass_ema_alpha   = COMPASS_EMA_ALPHA;
  input_ctx.compass_last_ms     = 0;
  input_ctx.compass_filter_init = false;
  input_ctx.compass_ready       = false;

  input_ctx.is_offset_mag_ready = false;
  input_ctx.initialized         = true;

  LOG_DBG("Init Battery");
  if (bsp_batt_init() != STATUS_OK)
  {
    LOG_ERR("Failed to initialize battery monitoring");
  }
  sys_input_initial_battery_level();

  LOG_DBG("Init ACC");
  if (bsp_acc_init() == STATUS_OK)
  {
    input_ctx.acc_ready = true;
    LOG_DBG("ACC OK");
  }

  // bsp_acc_config_interrupt(BSP_ACC_INT_PIN_1, BSP_ACC_INT_MOTION_DETECT);
  // bsp_io_int_init(ACC_INT_PIN, BSP_IO_EVENT_RISING, sys_input_trigger_io_event);

  LOG_DBG("Init GPS");
  if (bsp_gps_init(sys_input_gps_callback) == STATUS_OK)
  {
    input_ctx.gps_ready = true;
    LOG_DBG("GPS OK");
  }

  LOG_DBG("Init Dust");
  if (bsp_dust_sensor_init() == STATUS_OK)
  {
    input_ctx.dust_ready = true;
    LOG_DBG("Dust OK");
  }

  LOG_DBG("Init Compass");
  if (bsp_compass_init() == STATUS_OK)
  {
    input_ctx.compass_ready = true;
    LOG_DBG("Compass OK");
  }

  LOG_DBG("Init Temp/Hum");
  if (bsp_temp_hum_init() == STATUS_OK)
  {
    input_ctx.temp_hum_ready = true;
    LOG_DBG("Temp/Hum OK");
  }

  LOG_DBG("Calibrating — keep device stationary...");
  if (input_ctx.acc_ready)
  {
    (void) sys_input_calculate_offset_mag();
  }
}

status_function_t sys_input_process(void)
{
  if (!input_ctx.initialized)
    return STATUS_ERROR;

  size_t current_time_us = micros();
  size_t current_time_ms = millis();
  float  dt = (input_ctx.last_update_us == 0) ? 0.02f : (current_time_us - input_ctx.last_update_us) / US_TO_S;
  if (dt > 0.1f)
    dt = 0.1f;

  // 1. GPS: update velocity + anchor INS + Haversine distance
  sys_input_update_gps_data();

  // 2. GPS state machine: ACTIVE → FADING → INVALID
  sys_input_update_gps_state(current_time_ms);

  // 3. INS: 50Hz integration — decay rate depends on gps_state + is_stationary
  if (input_ctx.is_offset_mag_ready && input_ctx.acc_ready)
  {
    sys_input_update_ins_velocity(dt);
  }

  // 4. ZUPT: raw acc for fast stop detection
  sys_input_detect_zupt(input_ctx.acc_raw * GRAVITY_MS2, dt);

  // 5. Output
  sys_input_compute_output_velocity();

  // LOG_INF("V=%.3f GPS=%.3f INS=%.3f D=%.1f gps_state=%d stat=%d", input_ctx.data.velocity_ms, input_ctx.velocity_gps,
  //         input_ctx.velocity_ins, input_ctx.data.distance_m, input_ctx.gps_state, input_ctx.is_stationary);

  // 6. INS-only distance fallback when GPS unavailable
  if (input_ctx.gps_state == GPS_STATE_INVALID && input_ctx.is_offset_mag_ready && dt > 0.0f
      && input_ctx.data.velocity_ms > GPS_SPEED_MIN_MS)
  {
    input_ctx.data.distance_m += input_ctx.data.velocity_ms * dt;
  }

  // 7. Compass
  sys_input_read_compass(current_time_ms);

  // 8. Environmental sensors
  if ((current_time_ms - input_ctx.last_env_update_ms) >= SYS_INPUT_ENV_UPDATE_RATE_MS)
  {
    input_ctx.last_env_update_ms = current_time_ms;
    sys_input_read_dust_sensor();
    if (input_ctx.temp_hum_ready)
    {
      (void) bsp_temp_hum_read(&input_ctx.data.temp_hum);
    }
  }

  // 9. Battery level
  if ((current_time_ms - input_ctx.batt_last_update_ms) >= SYS_INPUT_BATT_UPDATE_RATE_MS)
  {
    input_ctx.batt_last_update_ms = current_time_ms;
    sys_input_read_battery_level(&input_ctx.data.battery_level);
  }

  // 10. Finalize
  input_ctx.data.timestamp_ms           = current_time_ms;
  input_ctx.last_update_us              = current_time_us;
  input_ctx.last_update_ms              = current_time_ms;
  input_ctx.data.gps_position.latitude  = input_ctx.gps_data_buffer.latitude;
  input_ctx.data.gps_position.longitude = input_ctx.gps_data_buffer.longitude;
  input_ctx.is_new_gps_data_available   = false;

  return STATUS_OK;
}

status_function_t sys_input_get_data(sys_input_data_t *data)
{
  if (data == NULL || !input_ctx.initialized)
    return STATUS_ERROR;
  *data = input_ctx.data;
  return STATUS_OK;
}

status_function_t sys_input_enter_sleep_mode(void)
{
  status_function_t status = STATUS_OK;

  status = bsp_acc_enable_interrupt(BSP_ACC_INT_PIN_1);
  return status;
}

/* Private definitions ----------------------------------------------- */

static status_function_t sys_input_calculate_offset_mag(void)
{
  if (!input_ctx.initialized)
    return STATUS_ERROR;

  float sum = 0.0f;
  for (uint16_t i = 0; i < ACC_OFFSET_MAGNITUDE_SAMPLE; i++)
  {
    bsp_acc_raw_data_t d = { 0 };
    if (bsp_acc_get_raw_data(&d) == STATUS_OK)
    {
      sum += sys_input_calculate_magnitude((float) d.acc_x, (float) d.acc_y, (float) d.acc_z);
    }
    delay(5);
  }

  input_ctx.offset_magnitude    = sum / (float) ACC_OFFSET_MAGNITUDE_SAMPLE;
  input_ctx.is_offset_mag_ready = true;
  LOG_DBG("Offset calibrated: %.4f", input_ctx.offset_magnitude);
  return STATUS_OK;
}

static float sys_input_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

/**
 * @brief Update INS velocity at 50Hz.
 *
 * Three-rate decay based on current state:
 *
 *   GPS_STATE_ACTIVE  + moving   → INS_DECAY_NORMAL   (slow, GPS keeps it accurate)
 *   GPS_STATE_FADING  + any      → INS_DECAY_GPS_LOST  (medium, smooth transition indoors)
 *   GPS_STATE_INVALID + moving   → INS_DECAY_NORMAL   (slow, pure INS mode)
 *   any state         + stationary → INS_DECAY_STOPPING (fast, ~0.5s to zero)
 *
 * Stationary always wins — stops immediately regardless of GPS state.
 */
static void sys_input_update_ins_velocity(float dt)
{
  bsp_acc_raw_data_t accel_data = { 0 };
  if (bsp_acc_get_raw_data(&accel_data) != STATUS_OK)
    return;

  float mag =
    sys_input_calculate_magnitude((float) accel_data.acc_x, (float) accel_data.acc_y, (float) accel_data.acc_z);
  float acc_clear = mag - input_ctx.offset_magnitude;

  input_ctx.acc_raw = acc_clear;  // Save raw for ZUPT

  input_ctx.acc_filtered = ACC_FILTER_ALPHA * acc_clear + (1.0f - ACC_FILTER_ALPHA) * input_ctx.acc_filtered;

  float accel_ms2 = input_ctx.acc_filtered * GRAVITY_MS2;

  if (fabsf(accel_ms2) > ACC_THRESHOLD)
  {
    input_ctx.velocity_ins += accel_ms2 * dt;
  }
  else
  {
    // Select decay rate based on state — stationary always takes priority
    float decay;
    if (input_ctx.is_stationary)
    {
      decay = INS_DECAY_STOPPING;  // Fast: ~0.5s to near-zero
    }
    else if (input_ctx.gps_state == GPS_STATE_FADING)
    {
      decay = INS_DECAY_GPS_LOST;  // Medium: smooth drop during GPS fade-out
    }
    else
    {
      decay = INS_DECAY_NORMAL;  // Slow: preserve velocity while moving
    }

    input_ctx.velocity_ins *= powf(decay, dt / 0.02f);
  }

  if (input_ctx.velocity_ins < 0.0f)
    input_ctx.velocity_ins = 0.0f;
}

/**
 * @brief Update GPS velocity and Haversine distance.
 *
 * When GPS is valid and moving:
 *   - EMA smooth the GPS speed
 *   - Re-anchor INS to GPS (GPS_ANCHOR_RATE=0.7 for higher sensitivity)
 *
 * When GPS reports stopped (raw_speed < threshold):
 *   - Zero GPS immediately (no EMA lag)
 *   - Pull INS toward zero via anchor
 *
 * GPS state transitions are handled in sys_input_update_gps_state().
 */
static void sys_input_update_gps_data(void)
{
  if (!input_ctx.gps_ready)
  {
    if (bsp_gps_init(sys_input_gps_callback) == STATUS_OK)
    {
      input_ctx.gps_ready = true;
    }
    else
    {
      return;
    }
  }

  if (!input_ctx.is_new_gps_data_available || !input_ctx.gps_data_buffer.location_valid)
  {
    return;
  }

  float raw_speed = input_ctx.gps_data_buffer.speed_kmph * KMH_TO_MS;
  bool  quality_ok =
    (input_ctx.gps_data_buffer.hdop < GPS_HDOP_MAX) && (input_ctx.gps_data_buffer.satellites >= GPS_SATELLITES_MIN);

  if (!quality_ok || raw_speed < GPS_SPEED_MIN_MS)
  {
    // GPS confirms stopped or poor quality — zero immediately, pull INS down
    input_ctx.velocity_gps = 0.0f;
    input_ctx.velocity_ins *= (1.0f - GPS_ANCHOR_RATE);
  }
  else
  {
    // Moving with good GPS — EMA smooth + re-anchor INS
    input_ctx.velocity_gps = GPS_EMA_ALPHA * raw_speed + (1.0f - GPS_EMA_ALPHA) * input_ctx.velocity_gps;

    // Re-anchor INS: GPS_ANCHOR_RATE=0.7 means INS is pulled 70% toward GPS each update
    // Higher value = more responsive to GPS changes, less INS drift accumulation
    input_ctx.velocity_ins =
      (1.0f - GPS_ANCHOR_RATE) * input_ctx.velocity_ins + GPS_ANCHOR_RATE * input_ctx.velocity_gps;
  }

  input_ctx.last_gps_ms = millis();

  // Haversine distance — only when quality OK and moving
  if (quality_ok)
  {
    float lat = input_ctx.gps_data_buffer.latitude;
    float lon = input_ctx.gps_data_buffer.longitude;

    if (input_ctx.has_last_gps_position)
    {
      float d = sys_input_haversine_m(input_ctx.last_valid_lat, input_ctx.last_valid_lon, lat, lon);
      if (d < GPS_MAX_STEP_M && input_ctx.velocity_gps > GPS_SPEED_MIN_MS)
      {
        input_ctx.data.distance_m += d;
      }
    }

    input_ctx.last_valid_lat        = lat;
    input_ctx.last_valid_lon        = lon;
    input_ctx.has_last_gps_position = true;
  }
}

/**
 * @brief GPS state machine — handles smooth outdoor→indoor transition.
 *
 * State transitions:
 *
 *   INVALID ──(GPS valid)──→ ACTIVE
 *   ACTIVE  ──(GPS lost)──→ FADING   ← new state, prevents hard jerk
 *   FADING  ──(fade done)──→ INVALID
 *   FADING  ──(GPS back)──→ ACTIVE
 *
 * During FADING (GPS_FADE_TIMEOUT_MS = 1000ms):
 *   - INS uses INS_DECAY_GPS_LOST instead of NORMAL
 *   - velocity_gps is already 0 (set in update_gps_data)
 *   - INS smoothly ramps down instead of holding then dropping hard
 *
 * This eliminates the jerk when walking from outdoor into a building.
 */
static void sys_input_update_gps_state(size_t current_ms)
{
  bool gps_recently_updated =
    (input_ctx.last_gps_ms > 0) && ((current_ms - input_ctx.last_gps_ms) < GPS_VALID_TIMEOUT_MS);

  switch (input_ctx.gps_state)
  {
  case GPS_STATE_INVALID:
    if (gps_recently_updated && input_ctx.velocity_gps > 0.0f)
    {
      input_ctx.gps_state = GPS_STATE_ACTIVE;
      LOG_DBG("GPS: INVALID → ACTIVE");
    }
    break;

  case GPS_STATE_ACTIVE:
    if (!gps_recently_updated)
    {
      // GPS just lost — start fade-out instead of immediate drop
      input_ctx.gps_state   = GPS_STATE_FADING;
      input_ctx.gps_lost_ms = current_ms;
      LOG_DBG("GPS: ACTIVE → FADING");
    }
    break;

  case GPS_STATE_FADING:
    if (gps_recently_updated && input_ctx.velocity_gps > 0.0f)
    {
      // GPS recovered — go back to active
      input_ctx.gps_state = GPS_STATE_ACTIVE;
      LOG_DBG("GPS: FADING → ACTIVE");
    }
    else if ((current_ms - input_ctx.gps_lost_ms) >= GPS_FADE_TIMEOUT_MS)
    {
      // Fade complete — fully invalidate
      input_ctx.gps_state    = GPS_STATE_INVALID;
      input_ctx.velocity_gps = 0.0f;
      LOG_DBG("GPS: FADING → INVALID");
    }
    break;

  default: input_ctx.gps_state = GPS_STATE_INVALID; break;
  }
}

/**
 * @brief ZUPT detection using pre-EMA acc_raw.
 *
 * is_stationary=true triggers INS_DECAY_STOPPING immediately in update_ins_velocity.
 * Hard-zero fires after ZUPT_TIME_THRESHOLD_MS confirmation.
 */
static void sys_input_detect_zupt(float accel_ms2, float dt)
{
  if (fabsf(accel_ms2) < ZUPT_ACC_THRESHOLD)
  {
    if (!input_ctx.is_stationary)
    {
      input_ctx.is_stationary      = true;
      input_ctx.stationary_time_ms = 0;
    }
    else
    {
      input_ctx.stationary_time_ms += (uint32_t) (dt * 1000.0f);
    }

    if (input_ctx.stationary_time_ms >= ZUPT_TIME_THRESHOLD_MS)
    {
      input_ctx.velocity_ins      = 0.0f;
      input_ctx.velocity_gps      = 0.0f;
      input_ctx.data.velocity_ms  = 0.0f;
      input_ctx.data.velocity_kmh = 0.0f;
    }
  }
  else
  {
    input_ctx.is_stationary      = false;
    input_ctx.stationary_time_ms = 0;
  }
}

/**
 * @brief Compute final output velocity from INS.
 *
 * Output = velocity_ins (50Hz smooth, GPS-anchored when available).
 * No complementary filter needed — INS already tracks GPS via GPS_ANCHOR_RATE.
 */
static void sys_input_compute_output_velocity(void)
{
  float output = input_ctx.velocity_ins;
  if (output < 0.0f)
    output = 0.0f;

  input_ctx.data.velocity_ms  = output;
  input_ctx.data.velocity_kmh = output * MS_TO_KMH;
}

static void sys_input_read_dust_sensor(void)
{
  if (!input_ctx.dust_ready)
  {
    input_ctx.data.dust_value = 0.0f;
    return;
  }

  bsp_dust_sensor_data_t dust_data;
  if (bsp_dust_sensor_read(&dust_data) != STATUS_OK)
    return;

  float new_value = (float) dust_data.running_average;
  input_ctx.data.dust_value =
    SYS_INPUT_DUST_EMA_ALPHA * new_value + (1.0f - SYS_INPUT_DUST_EMA_ALPHA) * input_ctx.data.dust_value;
  // LOG_INF("Dust: %.2f", input_ctx.data.dust_value);
}

static const char *s_direction_strings[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

static const char *sys_input_deg_to_direction_str(float deg)
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

static void sys_input_read_compass(size_t current_ms)
{
  if (!input_ctx.compass_ready)
    return;
  if ((current_ms - input_ctx.compass_last_ms) < COMPASS_UPDATE_MS)
    return;
  input_ctx.compass_last_ms = current_ms;

  bsp_compass_raw_data_t raw_data;
  if (bsp_compass_read_raw(&raw_data) != STATUS_OK)
  {
    LOG_ERR("Read compass fail");
    return;
  }

  float alpha = input_ctx.compass_ema_alpha;
  if (!input_ctx.compass_filter_init)
  {
    input_ctx.compass_ema_x       = (float) raw_data.raw_x;
    input_ctx.compass_ema_y       = (float) raw_data.raw_y;
    input_ctx.compass_ema_z       = (float) raw_data.raw_z;
    input_ctx.compass_filter_init = true;
  }
  else
  {
    input_ctx.compass_ema_x = alpha * (float) raw_data.raw_x + (1.0f - alpha) * input_ctx.compass_ema_x;
    input_ctx.compass_ema_y = alpha * (float) raw_data.raw_y + (1.0f - alpha) * input_ctx.compass_ema_y;
    input_ctx.compass_ema_z = alpha * (float) raw_data.raw_z + (1.0f - alpha) * input_ctx.compass_ema_z;
  }

  float heading_rad = atan2f(input_ctx.compass_ema_y, input_ctx.compass_ema_x);
  float heading_deg = heading_rad * 180.0f / (float) M_PI;
  if (heading_deg < 0.0f)
    heading_deg += 360.0f;

  input_ctx.data.heading_deg   = heading_deg;
  input_ctx.data.direction_str = sys_input_deg_to_direction_str(heading_deg);
}

static void sys_input_gps_callback(bsp_gps_data_t *data)
{
  if (data == NULL)
    return;
  input_ctx.gps_data_buffer           = *data;
  input_ctx.is_new_gps_data_available = true;
  input_ctx.last_gps_ms               = millis();
}

static float sys_input_haversine_m(float lat1, float lon1, float lat2, float lon2)
{
  const float R    = 6371000.0f;
  float       dlat = (lat2 - lat1) * (float) M_PI / 180.0f;
  float       dlon = (lon2 - lon1) * (float) M_PI / 180.0f;
  float       a =
    sinf(dlat / 2.0f) * sinf(dlat / 2.0f)
    + cosf(lat1 * (float) M_PI / 180.0f) * cosf(lat2 * (float) M_PI / 180.0f) * sinf(dlon / 2.0f) * sinf(dlon / 2.0f);
  return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

static void sys_input_trigger_io_event(void)
{
  Serial.println("Triggering IO event");
}

static void sys_input_read_battery_level(float *battery_level)
{
  static uint8_t err_cnt = 0;
  uint32_t       now     = OS_GET_TICK();
  size_t         dt_ms   = now - input_ctx.batt_last_update_ms;

  float raw_ma   = bsp_batt_read_current_ma();
  *battery_level = (SYS_INPUT_BATT_EMA_ALPHA * raw_ma) + ((1.0f - SYS_INPUT_BATT_EMA_ALPHA) * (*battery_level));

  float delta_mah = (*battery_level) * (dt_ms / 3600000.0f);
  input_ctx.batt_remaining_mah -= delta_mah;
  input_ctx.batt_remaining_mah = input_ctx.batt_remaining_mah < 0.0f ? 0.0f
                                 : input_ctx.batt_remaining_mah > BSP_BATTERY_CAPACITY_MAH
                                   ? BSP_BATTERY_CAPACITY_MAH
                                   : input_ctx.batt_remaining_mah;

  float   voltage_mv      = 0.0f;
  float   sum_voltage_mv  = 0.0f;
  uint8_t check_err_count = 0;
  for (int i = 0; i < SYS_INPUT_BATT_READ_VOLT_TIMES; i++)
  {
    voltage_mv = bsp_batt_read_voltage_mv();

    if (voltage_mv <= 0.0f)
    {
      LOG_ERR("Battery voltage read 0 volt");
      check_err_count++;
    }
    else
    {
      sum_voltage_mv += voltage_mv;
    }

    if (check_err_count > SYS_INPUT_BATT_DEBOUNCE)
    {
      // Not update battery level if too many read errors
      err_cnt++;
      if (err_cnt > SYS_INPUT_BATT_MAX_ERROR)
      {
        err_cnt = 0;
        LOG_ERR("Too many consecutive battery read errors, resetting remaining capacity to 0");
        // Handle error battery level if too many consecutive errors
      }
      return;
    }
    OS_YIELD();
  }
  voltage_mv = sum_voltage_mv / ((float) SYS_INPUT_BATT_READ_VOLT_TIMES);

  if (voltage_mv >= BSP_BATT_VOLTAGE_FULL_MV)
  {
    input_ctx.batt_remaining_mah = BSP_BATTERY_CAPACITY_MAH;
    *battery_level               = 100.0f;
    return;
  }
  if (voltage_mv <= BSP_BATT_VOLTAGE_EMPTY_MV)
  {
    input_ctx.batt_remaining_mah = 0.0f;
    *battery_level               = 0.0f;
    return;
  }

  float soc_coulomb = (input_ctx.batt_remaining_mah / BSP_BATTERY_CAPACITY_MAH) * 100.0f;
  float soc_voltage =
    ((float) (voltage_mv - BSP_BATT_VOLTAGE_EMPTY_MV) / (float) (BSP_BATT_VOLTAGE_FULL_MV - BSP_BATT_VOLTAGE_EMPTY_MV))
    * 100.0f;

  float soc = (0.9f * soc_coulomb) + (0.1f * soc_voltage);
  soc       = soc < 0.0f ? 0.0f : soc > 100.0f ? 100.0f : soc;

  *battery_level = soc;
}

static void sys_input_initial_battery_level(void)
{
  float sum = 0.0f;

  for (int i = 0; i < SYS_INPUT_BATT_INITIAL_SAMPLES; i++)
  {
    int32_t m_volt = bsp_batt_read_voltage_mv();

    if (m_volt >= BSP_BATT_VOLTAGE_FULL_MV)
      sum += 100.0f;
    else if (m_volt <= BSP_BATT_VOLTAGE_EMPTY_MV)
      sum += 0.0f;
    else
      sum +=
        ((float) (m_volt - BSP_BATT_VOLTAGE_EMPTY_MV) / (float) (BSP_BATT_VOLTAGE_FULL_MV - BSP_BATT_VOLTAGE_EMPTY_MV))
        * 100.0f;

    OS_YIELD();
  }

  float initial_soc = sum / SYS_INPUT_BATT_INITIAL_SAMPLES;

  input_ctx.batt_remaining_mah = (initial_soc / 100.0f) * BSP_BATTERY_CAPACITY_MAH;
  input_ctx.data.battery_level = initial_soc;

  LOG_INF("Initial battery level: %.2f%%", initial_soc);
}

/* End of file -------------------------------------------------------- */