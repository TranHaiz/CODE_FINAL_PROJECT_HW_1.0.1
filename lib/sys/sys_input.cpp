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
 */

/* Includes ----------------------------------------------------------- */
#include "sys_input.h"

#include "bsp_acc.h"
#include "bsp_compass.h"
#include "bsp_dust_sensor.h"
#include "bsp_gps.h"
#include "bsp_temp_hum.h"
#include "log_service.h"

#include <math.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_input, LOG_LEVEL_WARN)
// Acc parameters
#define ACC_FILTER_ALPHA            (0.15f)
#define ACC_THRESHOLD               (0.30f)
#define ACC_OFFSET_MAGNITUDE_SAMPLE (200)
#define INS_DRIFT_RATE              (0.02f)
#define ZUPT_ACC_THRESHOLD          (0.03f)
#define ZUPT_TIME_THRESHOLD_MS      (1000)

// Compass parameters
#define COMPASS_EMA_ALPHA           (0.15f)
#define COMPASS_UPDATE_MS           (100)

#define GRAVITY_MS2                 (9.806f)
#define COMP_FILTER_K               (0.92f)
#define GPS_VALID_TIMEOUT_MS        (2000)

#define INS_DECAY_PER_SECOND        (0.98f)

#define KMH_TO_MS                   (1.0f / 3.6f)
#define MS_TO_KMH                   (3.6f)
#define US_TO_S                     (1000000.0f)

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Private context structure
 */
typedef struct
{
  // GPS data management
  bool           is_new_gps_data_available;
  bsp_gps_data_t gps_data_buffer;
  size_t         last_gps_ms;
  float          velocity_gps;

  size_t last_update_ms;
  size_t last_update_us;
  float  velocity_ins;
  float  acc_filtered;
  float  offset_magnitude;

  /* Zero-Velocity Update (ZUPT) Detection */
  bool     is_stationary;
  uint32_t stationary_time_ms;

  /* Compass filter state */
  float  compass_ema_x;
  float  compass_ema_y;
  float  compass_ema_z;
  float  compass_ema_alpha;
  size_t compass_last_ms;
  bool   compass_filter_init;

  bool compass_ready;
  bool acc_ready;
  bool gps_ready;

  /* State tracking */
  bool     dust_ready;
  bool     temp_hum_ready;
  bool     initialized;
  bool     is_offset_mag_ready;
  uint32_t last_env_update_ms;

  // temporary data for output
  sys_input_data_t data;  // Current sensor data
} sys_input_context_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sys_input_context_t input_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static float             sys_input_calculate_magnitude(float x, float y, float z);
static status_function_t sys_input_calculate_offset_mag(void);
static void              sys_input_update_ins_velocity(float dt);
static void              sys_input_update_gps_data(void);
static void              sys_input_detect_zupt(float accel_ms2, float dt);
static void              sys_input_fuse_velocity_gps_ins(void);
static void              sys_input_read_dust_sensor(void);
static void              sys_input_read_compass(size_t current_ms);
static const char       *sys_input_deg_to_direction_str(float deg);
static void              sys_input_gps_callback(bsp_gps_data_t *data);

/* Function definitions ----------------------------------------------- */

/**
 * @brief Initialize system input with sensors
 */
void sys_input_init(void)
{
  if (input_ctx.initialized)
  {
    return;
  }

  LOG_DBG("Initializing system input...");

  // Initialize data structure
  input_ctx.data.velocity_ms          = 0.0f;
  input_ctx.data.velocity_kmh         = 0.0f;
  input_ctx.data.distance_m           = 0.0f;
  input_ctx.data.dust_value           = 0.0f;
  input_ctx.data.temp_hum.temperature = 0.0f;
  input_ctx.data.temp_hum.humidity    = 0.0f;
  input_ctx.data.timestamp_ms         = 0;
  input_ctx.last_env_update_ms        = 0;

  // Initialize state variables
  input_ctx.last_update_ms   = 0;
  input_ctx.last_update_us   = 0;
  input_ctx.last_gps_ms      = 0;
  input_ctx.velocity_ins     = 0.0f;
  input_ctx.velocity_gps     = 0.0f;
  input_ctx.acc_filtered     = 0.0f;
  input_ctx.offset_magnitude = 0.0f;
  /* ZUPT initialization */
  input_ctx.is_stationary      = false;
  input_ctx.stationary_time_ms = 0;
  input_ctx.acc_ready          = false;
  input_ctx.gps_ready          = false;
  input_ctx.dust_ready         = false;
  /* Compass initialization */
  input_ctx.data.heading_deg    = 0.0f;
  input_ctx.data.direction_str  = "N";
  input_ctx.compass_ema_x       = 0.0f;
  input_ctx.compass_ema_y       = 0.0f;
  input_ctx.compass_ema_z       = 0.0f;
  input_ctx.compass_ema_alpha   = COMPASS_EMA_ALPHA;
  input_ctx.compass_last_ms     = 0;
  input_ctx.compass_filter_init = false;
  input_ctx.compass_ready       = false;
  /* State */
  input_ctx.is_offset_mag_ready = false;
  input_ctx.initialized         = true;

  LOG_DBG("Init ACC");
  if (bsp_acc_init() == STATUS_OK)
  {
    LOG_DBG("ACC initialized successfully");
    input_ctx.acc_ready = true;
  }

  LOG_DBG("Init GPS");
  if (bsp_gps_init(sys_input_gps_callback) == STATUS_OK)
  {
    LOG_DBG("GPS initialized successfully");
    input_ctx.gps_ready = true;
  }

  LOG_DBG("Init Dust Sensor");
  if (bsp_dust_sensor_init() == STATUS_OK)
  {
    LOG_DBG("Dust sensor initialized successfully");
    input_ctx.dust_ready = true;
  }

  LOG_DBG("Init Compass");
  if (bsp_compass_init() == STATUS_OK)
  {
    LOG_DBG("Compass initialized successfully");
    input_ctx.compass_ready = true;
  }

  LOG_DBG("Init Temp/Hum Sensor");
  if (bsp_temp_hum_init() == STATUS_OK)
  {
    LOG_DBG("Temp/Hum sensor initialized successfully");
    input_ctx.temp_hum_ready = true;
  }

  LOG_DBG("Start calibration - keep device stationary...");
  // Perform one-shot calibration at init when sensor is stationary.
  // NOTE: This call blocks for ~1 second (200 samples x 5 ms delay).
  //       Must be called from a context that allows blocking (e.g. main loop / init task).
  if (input_ctx.acc_ready)
  {
    (void) sys_input_calculate_offset_mag();
  }
}

/**
 * @brief Process sensor data
 */
status_function_t sys_input_process(void)
{
  if (!input_ctx.initialized)
  {
    return STATUS_ERROR;
  }

  // Capture timestamps once per cycle to keep all sub-functions consistent
  size_t current_time_us = micros();
  size_t current_time_ms = millis();
  float  dt = (input_ctx.last_update_us == 0) ? 0.02f : (current_time_us - input_ctx.last_update_us) / US_TO_S;

  // Limit dt to avoid large jumps
  if (dt > 0.1f)
  {
    dt = 0.1f;
  }

  // Update GPS data
  sys_input_update_gps_data();

  if (input_ctx.is_offset_mag_ready && input_ctx.acc_ready)
  {
    sys_input_update_ins_velocity(dt);
  }

  input_ctx.data.velocity_ms  = input_ctx.velocity_ins;
  input_ctx.data.velocity_kmh = input_ctx.data.velocity_ms * MS_TO_KMH;

  // // Detect ZUPT state to correct INS drift
  // sys_input_detect_zupt(input_ctx.acc_filtered * GRAVITY_MS2, dt);
  // // Calculate velocity using GPS and INS data
  // sys_input_fuse_velocity_gps_ins();

  // Update distance — only when INS is calibrated and velocity is meaningful
  if (input_ctx.is_offset_mag_ready && input_ctx.data.velocity_ms > 0.01f && dt > 0.0f)
  {
    input_ctx.data.distance_m += input_ctx.data.velocity_ms * dt;
  }

  // Read compass — pass current_time_ms to avoid redundant millis() call
  sys_input_read_compass(current_time_ms);

  // Read environmental sensors at configured rate
  if ((current_time_ms - input_ctx.last_env_update_ms) >= SYS_INPUT_ENV_UPDATE_RATE_MS)
  {
    input_ctx.last_env_update_ms = current_time_ms;  // FIX: update timestamp to enforce rate limiting

    sys_input_read_dust_sensor();
    if (input_ctx.temp_hum_ready)
    {
      if (bsp_temp_hum_read(&input_ctx.data.temp_hum) == STATUS_OK)
      {
        // Do nothing
      }
    }
  }

  // Summarize data for output
  input_ctx.data.timestamp_ms           = current_time_ms;
  input_ctx.last_update_us              = current_time_us;
  input_ctx.last_update_ms              = current_time_ms;
  input_ctx.data.gps_position.latitude  = input_ctx.gps_data_buffer.latitude;
  input_ctx.data.gps_position.longitude = input_ctx.gps_data_buffer.longitude;
  input_ctx.is_new_gps_data_available   = false;

  return STATUS_OK;
}

/**
 * @brief Get current sensor input data
 */
status_function_t sys_input_get_data(sys_input_data_t *data)
{
  if (data == NULL || !input_ctx.initialized)
  {
    return STATUS_ERROR;
  }

  *data = input_ctx.data;
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */

/**
 * @brief Calibrate system (set gravity offset)
 * @note  Must be called once on stationary device to calibrate IMU.
 *        Blocks for approximately (ACC_OFFSET_MAGNITUDE_SAMPLE * 5) ms.
 */
static status_function_t sys_input_calculate_offset_mag(void)
{
  if (!input_ctx.initialized)
  {
    return STATUS_ERROR;
  }

  // Take multiple samples to average gravity offset
  float sum_magnitude = 0.0f;

  for (uint16_t i = 0; i < ACC_OFFSET_MAGNITUDE_SAMPLE; i++)
  {
    bsp_acc_raw_data_t accel_data = { 0 };
    if (bsp_acc_get_raw_data(&accel_data) == STATUS_OK)
    {
      sum_magnitude +=
        sys_input_calculate_magnitude((float) accel_data.acc_x, (float) accel_data.acc_y, (float) accel_data.acc_z);
    }
    delay(5);
  }

  input_ctx.offset_magnitude    = sum_magnitude / (float) ACC_OFFSET_MAGNITUDE_SAMPLE;
  input_ctx.is_offset_mag_ready = true;

  return STATUS_OK;
}

/**
 * @brief Calculate magnitude of acceleration vector
 */
static float sys_input_calculate_magnitude(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

/**
 * @brief Update INS velocity by integrating acceleration
 */
static void sys_input_update_ins_velocity(float dt)
{
  // Read raw IMU data
  bsp_acc_raw_data_t accel_data = { 0 };

  if (bsp_acc_get_raw_data(&accel_data) != STATUS_OK)
  {
    return;
  }

  // Calculate current acceleration magnitude
  float current_magnitude = sys_input_calculate_magnitude(accel_data.acc_x, accel_data.acc_y, accel_data.acc_z);

  // Remove gravity offset
  float acc_clear = current_magnitude - input_ctx.offset_magnitude;

  // Apply low-pass filter to acceleration
  input_ctx.acc_filtered = ACC_FILTER_ALPHA * acc_clear + (1.0f - ACC_FILTER_ALPHA) * input_ctx.acc_filtered;

  // Convert to m/s²
  float accel_ms2 = input_ctx.acc_filtered * GRAVITY_MS2;

  // Integrate to calculate INS velocity
  if (fabsf(accel_ms2) > ACC_THRESHOLD)
  {
    input_ctx.velocity_ins += accel_ms2 * dt;
  }
  else
  {
    // Apply dt-normalized velocity decay when no significant acceleration (INS drift correction).
    // powf(INS_DECAY_PER_SECOND, dt / 0.02f) normalizes the decay to be frequency-independent:
    // at 50 Hz (dt=0.02s) decay = 0.98 per tick; at 10 Hz (dt=0.1s) decay = 0.98^5 ≈ 0.904 per tick.
    input_ctx.velocity_ins *= powf(INS_DECAY_PER_SECOND, dt / 0.02f);
  }

  // Ensure INS velocity is not negative
  if (input_ctx.velocity_ins < 0.0f)
  {
    input_ctx.velocity_ins = 0.0f;
  }
}

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

  if (input_ctx.is_new_gps_data_available && input_ctx.gps_data_buffer.location_valid)
  {
    // Update GPS velocity (convert km/h to m/s)
    input_ctx.velocity_gps = (float) (input_ctx.gps_data_buffer.speed_kmph * KMH_TO_MS);

    // Update timestamp on valid GPS sample
    input_ctx.last_gps_ms = millis();
  }
  else
  {
    input_ctx.velocity_gps = 0.0f;
  }
}

/**
 * @brief Detect Zero-Velocity Update (ZUPT) state
 * @param accel_ms2  Filtered acceleration magnitude in m/s²
 * @param dt         Time delta in seconds (used for accurate time accumulation)
 *
 * Used to recognize stationary periods and reduce INS drift
 */
static void sys_input_detect_zupt(float accel_ms2, float dt)
{
  if (fabsf(accel_ms2) < ZUPT_ACC_THRESHOLD)
  {
    // Device is stationary
    if (!input_ctx.is_stationary)
    {
      input_ctx.is_stationary      = true;
      input_ctx.stationary_time_ms = 0;
    }
    else
    {
      // FIX: accumulate real elapsed time instead of assuming fixed 20 ms tick
      input_ctx.stationary_time_ms += (uint32_t) (dt * 1000.0f);
    }

    // If confirmed stationary for threshold time, reset INS velocity
    if (input_ctx.stationary_time_ms >= ZUPT_TIME_THRESHOLD_MS)
    {
      // ZUPT correction: reset velocity drift
      input_ctx.velocity_ins = 0.0f;
    }
  }
  else
  {
    // Device is moving
    input_ctx.is_stationary      = false;
    input_ctx.stationary_time_ms = 0;
  }
}

/**
 * @brief GPS/INS fusion with fixed complementary filter
 *
 * Simpler but still robust:
 * - GPS gives absolute speed reference
 * - INS gives short-term smooth speed
 * - Fixed k blends both: v = k*v_gps + (1-k)*v_ins
 */
static void sys_input_fuse_velocity_gps_ins(void)
{
  float fused_velocity = 0.0f;

  // Check if GPS is recent and valid
  bool   gps_valid = false;
  size_t now_ms    = millis();
  if (input_ctx.last_gps_ms > 0)
  {
    size_t gps_age = now_ms - input_ctx.last_gps_ms;
    gps_valid      = (gps_age < GPS_VALID_TIMEOUT_MS) && (input_ctx.velocity_gps > 0.0f);
  }

  if (gps_valid && input_ctx.is_offset_mag_ready)
  {
    // Apply fixed complementary filter fusion
    float k        = COMP_FILTER_K;
    fused_velocity = k * input_ctx.velocity_gps + (1.0f - k) * input_ctx.velocity_ins;

    // Lightly re-anchor INS to fused result to limit long-term drift.
    input_ctx.velocity_ins = fused_velocity;
  }
  else if (input_ctx.is_offset_mag_ready)
  {
    // GPS not available: use INS only.
    fused_velocity = input_ctx.velocity_ins;
  }
  else
  {
    fused_velocity = 0.0f;
  }

  // Ensure velocity is not negative
  if (fused_velocity < 0.0f)
  {
    fused_velocity = 0.0f;
  }

  // Update fused velocity
  input_ctx.data.velocity_ms  = fused_velocity;
  input_ctx.data.velocity_kmh = fused_velocity * MS_TO_KMH;
}

/**
 * @brief Read dust sensor data
 */
static void sys_input_read_dust_sensor(void)
{
  if (!input_ctx.dust_ready)
  {
    input_ctx.data.dust_value = 0.0f;
    return;
  }

  bsp_dust_sensor_data_t dust_data;
  if (bsp_dust_sensor_read(&dust_data) == STATUS_OK)
  {
    input_ctx.data.dust_value = (float) dust_data.dust_density;
  }
}

/**
 * @brief Direction strings for compass
 */
static const char *s_direction_strings[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

/**
 * @brief Convert degrees to cardinal direction string
 */
static const char *sys_input_deg_to_direction_str(float deg)
{
  uint8_t index = 0;
  if (deg < 22.5f || deg >= 337.5f)
    index = 0;  // N
  else if (deg < 67.5f)
    index = 1;  // NE
  else if (deg < 112.5f)
    index = 2;  // E
  else if (deg < 157.5f)
    index = 3;  // SE
  else if (deg < 202.5f)
    index = 4;  // S
  else if (deg < 247.5f)
    index = 5;  // SW
  else if (deg < 292.5f)
    index = 6;  // W
  else
    index = 7;  // NW

  return s_direction_strings[index];
}

/**
 * @brief Read and filter compass data
 * @param current_ms  Current timestamp in ms (passed in to avoid redundant millis() call)
 *
 * Applies EMA (Exponential Moving Average) filter to raw compass data
 * and calculates heading in degrees with cardinal direction.
 */
static void sys_input_read_compass(size_t current_ms)
{
  if (!input_ctx.compass_ready)
  {
    return;
  }

  // Rate limiting: update at configured interval
  if ((current_ms - input_ctx.compass_last_ms) < COMPASS_UPDATE_MS)
  {
    return;
  }
  input_ctx.compass_last_ms = current_ms;

  // Read raw data from sensor
  bsp_compass_raw_data_t raw_data;
  if (bsp_compass_read_raw(&raw_data) != STATUS_OK)
  {
    LOG_ERR("Read compass fail");
    return;
  }

  // Apply EMA filter
  // FIX: use local 'alpha' consistently for both multiply terms instead of mixing
  //      the local variable with the COMPASS_EMA_ALPHA constant.
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
    input_ctx.compass_ema_x = alpha * raw_data.raw_x + (1.0f - alpha) * input_ctx.compass_ema_x;
    input_ctx.compass_ema_y = alpha * raw_data.raw_y + (1.0f - alpha) * input_ctx.compass_ema_y;
    input_ctx.compass_ema_z = alpha * raw_data.raw_z + (1.0f - alpha) * input_ctx.compass_ema_z;
  }

  // Calculate heading from filtered X and Y
  float heading_rad = atan2f(input_ctx.compass_ema_y, input_ctx.compass_ema_x);
  float heading_deg = heading_rad * 180.0f / M_PI;

  // Normalize to 0-360 degrees
  if (heading_deg < 0.0f)
  {
    heading_deg += 360.0f;
  }

  // Update main data structure
  input_ctx.data.heading_deg   = heading_deg;
  input_ctx.data.direction_str = sys_input_deg_to_direction_str(heading_deg);
}

static void sys_input_gps_callback(bsp_gps_data_t *data)
{
  if (data == NULL)
  {
    return;
  }

  // FIX: use the data pointer passed by BSP directly instead of calling
  //      bsp_gps_get_data() again, which is redundant and risks a race condition.
  input_ctx.gps_data_buffer           = *data;
  input_ctx.is_new_gps_data_available = true;
  input_ctx.last_gps_ms               = millis();
}

/* End of file -------------------------------------------------------- */