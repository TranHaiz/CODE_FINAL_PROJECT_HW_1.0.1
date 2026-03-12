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
LOG_MODULE_REGISTER(sys_input, LOG_LEVEL_DBG)
#define ACC_FILTER_ALPHA       (0.15f)   // Low-pass filter for acceleration
#define ACC_THRESHOLD          (0.20f)   // Vehicle tune: reduce accel noise integration
#define COMP_FILTER_K          (0.92f)   // Vehicle tune: trust GPS more for stable speed
#define GRAVITY_MS2            (9.806f)  // Earth gravity in m/s²
#define GPS_VALID_TIMEOUT_MS   (2000)    // GPS data validity timeout
#define INS_DRIFT_RATE         (0.02f)   // INS drift rate per second (m/s per s)
#define ZUPT_ACC_THRESHOLD     (0.03f)   // Vehicle tune: avoid false stationary at cruise
#define ZUPT_TIME_THRESHOLD_MS (1000)    // Time for ZUPT confirmation (ms)

/* Compass filter settings */
#define COMPASS_EMA_ALPHA      (0.15f)  // EMA filter coefficient for compass
#define COMPASS_UPDATE_MS      (100)    // Compass update rate (10Hz)

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Private context structure
 */
typedef struct
{
  // GPS data management
  bool           is_new_gps_data_available;
  bsp_gps_data_t gps_data_buffer;  // Buffer for latest GPS data
  size_t         last_gps_ms;      // Last valid GPS timestamp
  float          velocity_gps;     // GPS-derived velocity

  sys_input_data_t data;              // Current sensor data
  size_t           last_update_ms;    // Last update timestamp
  size_t           last_update_us;    // Last update timestamp (microseconds)
  float            velocity_ins;      // INS-derived velocity
  float            acc_filtered;      // Filtered acceleration
  float            offset_magnitude;  // Gravity offset
  /* Zero-Velocity Update (ZUPT) Detection */
  bool   is_stationary;       // Stationary state flag
  size_t stationary_time_ms;  // Duration in stationary state (ms)
  /* Compass filter state */
  float  compass_ema_x;        // EMA filtered X
  float  compass_ema_y;        // EMA filtered Y
  float  compass_ema_z;        // EMA filtered Z
  float  compass_ema_alpha;    // EMA filter coefficient
  size_t compass_last_ms;      // Last compass update timestamp
  bool   compass_filter_init;  // Filter initialized flag
  /* State tracking */
  bool acc_ready;       // Accelerometer ready flag
  bool gps_ready;       // GPS ready flag
  bool dust_ready;      // Dust sensor ready flag
  bool temp_hum_ready;  // Temperature and humidity sensor ready flag
  bool compass_ready;   // Compass ready flag
  bool initialized;     // Initialization flag
  bool is_calibrated;   // Calibration flag
} sys_input_context_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sys_input_context_t input_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static float             sys_input_calculate_magnitude(float x, float y, float z);
static status_function_t sys_input_calibrate_internal(void);
static void              sys_input_update_ins_velocity(float dt);
static void              sys_input_update_gps_data(void);
static void              sys_input_detect_zupt(float accel_ms2);
static void              sys_input_fuse_velocity_gps_ins(void);
static void              sys_input_read_dust_sensor(void);
static void              sys_input_read_compass(void);
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
  input_ctx.data.dust_concentration   = 0.0f;
  input_ctx.data.temp_hum.temperature = 0.0f;
  input_ctx.data.temp_hum.humidity    = 0.0f;
  input_ctx.data.timestamp_ms         = 0;

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
  input_ctx.is_calibrated = false;
  input_ctx.initialized   = true;

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
  if (input_ctx.acc_ready)
  {
    (void) sys_input_calibrate_internal();
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

  // Get current timestamp
  size_t current_time_us = micros();
  size_t current_time_ms = millis();
  float  dt = (input_ctx.last_update_us == 0) ? 0.02f : (current_time_us - input_ctx.last_update_us) / 1000000.0f;

  // Limit dt to avoid large jumps
  if (dt > 0.1f)
  {
    dt = 0.1f;
  }

  // Update GPS data
  sys_input_update_gps_data();

  // Update INS velocity if calibrated
  if (input_ctx.is_calibrated && input_ctx.acc_ready)
  {
    sys_input_update_ins_velocity(dt);
  }

  // Detect stationary state (ZUPT)
  sys_input_detect_zupt(input_ctx.acc_filtered * GRAVITY_MS2);

  // Perform GPS/INS fusion with fixed complementary weight
  sys_input_fuse_velocity_gps_ins();

  // Update distance based on fused velocity
  if (input_ctx.data.velocity_ms > 0.01f && dt > 0.0f)
  {
    input_ctx.data.distance_m += input_ctx.data.velocity_ms * dt;
  }

  // Read dust sensor
  sys_input_read_dust_sensor();

  // Read compass
  sys_input_read_compass();

  // Read temperature and humidity data
  if (input_ctx.temp_hum_ready)
  {
    if (bsp_temp_hum_read(&input_ctx.data.temp_hum) == STATUS_OK)
    {
      // Do nothing
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
 * @note  Must be called once on stationary device to calibrate IMU
 */
static status_function_t sys_input_calibrate_internal(void)
{
  if (!input_ctx.initialized)
  {
    return STATUS_ERROR;
  }

  // Take multiple samples to average gravity offset
  float    sum_magnitude = 0.0f;
  uint16_t samples       = 50;

  for (uint16_t i = 0; i < samples; i++)
  {
    bsp_acc_raw_data_t accel_data = { 0 };
    if (bsp_acc_get_raw_data(&accel_data) == STATUS_OK)
    {
      sum_magnitude += sys_input_calculate_magnitude((float) accel_data.accel_x, (float) accel_data.accel_y,
                                                     (float) accel_data.accel_z);
    }
    delay(10);
  }

  input_ctx.offset_magnitude = sum_magnitude / (float) samples;
  input_ctx.is_calibrated    = true;

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
  float current_magnitude = sys_input_calculate_magnitude(accel_data.accel_x, accel_data.accel_y, accel_data.accel_z);

  // Remove gravity offset
  float accel_clear = current_magnitude - input_ctx.offset_magnitude;

  // Apply low-pass filter to acceleration
  input_ctx.acc_filtered = ACC_FILTER_ALPHA * accel_clear + (1.0f - ACC_FILTER_ALPHA) * input_ctx.acc_filtered;

  // Convert to m/s²
  float accel_ms2 = input_ctx.acc_filtered * GRAVITY_MS2;

  // Integrate to calculate INS velocity
  if (fabsf(accel_ms2) > ACC_THRESHOLD)
  {
    input_ctx.velocity_ins += accel_ms2 * dt;
  }
  else
  {
    // Apply velocity decay when no significant acceleration (INS drift correction)
    input_ctx.velocity_ins *= (1.0f - INS_DRIFT_RATE * dt);
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
    input_ctx.velocity_gps = (float) (input_ctx.gps_data_buffer.speed_kmph / 3.6);

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
 * Used to recognize stationary periods and reduce INS drift
 */
static void sys_input_detect_zupt(float accel_ms2)
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
      input_ctx.stationary_time_ms += 20;  // Assuming 50Hz update rate
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

  if (gps_valid && input_ctx.is_calibrated)
  {
    // Apply fixed complementary filter fusion
    float k        = COMP_FILTER_K;
    fused_velocity = k * input_ctx.velocity_gps + (1.0f - k) * input_ctx.velocity_ins;

    // Lightly re-anchor INS to fused result to limit long-term drift.
    input_ctx.velocity_ins = fused_velocity;
  }
  else if (input_ctx.is_calibrated)
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
  input_ctx.data.velocity_kmh = fused_velocity * 3.6f;
}

/**
 * @brief Read dust sensor data
 */
static void sys_input_read_dust_sensor(void)
{
  if (!input_ctx.dust_ready)
  {
    input_ctx.data.dust_concentration = 0.0f;
    return;
  }

  bsp_dust_sensor_data_t dust_data;
  if (bsp_dust_sensor_read(&dust_data) == STATUS_OK)
  {
    input_ctx.data.dust_concentration = (float) dust_data.dust_density;
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
 *
 * Applies EMA (Exponential Moving Average) filter to raw compass data
 * and calculates heading in degrees with cardinal direction.
 */
static void sys_input_read_compass(void)
{
  if (!input_ctx.compass_ready)
  {
    return;
  }

  // Rate limiting: update at configured interval
  size_t current_ms = millis();
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
    input_ctx.compass_ema_x = alpha * raw_data.raw_x + (1.0f - COMPASS_EMA_ALPHA) * input_ctx.compass_ema_x;
    input_ctx.compass_ema_y = alpha * raw_data.raw_y + (1.0f - COMPASS_EMA_ALPHA) * input_ctx.compass_ema_y;
    input_ctx.compass_ema_z = alpha * raw_data.raw_z + (1.0f - COMPASS_EMA_ALPHA) * input_ctx.compass_ema_z;
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
  input_ctx.is_new_gps_data_available = true;
  bsp_gps_get_data(&input_ctx.gps_data_buffer);
  input_ctx.last_gps_ms = millis();
}

/* End of file -------------------------------------------------------- */
