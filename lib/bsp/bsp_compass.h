/**
 * @file       bsp_compass.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP for HMC5883L Digital Compass Sensor
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_COMPASS_H_
#define _BSP_COMPASS_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <Wire.h>

/* Public defines ----------------------------------------------------- */
#define BSP_COMPASS_DEFAULT_I2C_ADDR       (0x1E)
#define BSP_COMPASS_DEFAULT_SDA_PIN        (4)
#define BSP_COMPASS_DEFAULT_SCL_PIN        (5)
#define BSP_COMPASS_DEFAULT_I2C_CLOCK      (100000)
#define BSP_COMPASS_DEFAULT_EMA_ALPHA      (0.15f)  // EMA filter coefficient
#define BSP_COMPASS_DEFAULT_UPDATE_RATE_MS (100)    // 10Hz update rate

// HMC5883L Register addresses
#define BSP_COMPASS_REG_CONFIG_A           (0x00)
#define BSP_COMPASS_REG_CONFIG_B           (0x01)
#define BSP_COMPASS_REG_MODE               (0x02)
#define BSP_COMPASS_REG_DATA_X_MSB         (0x03)
#define BSP_COMPASS_REG_DATA_X_LSB         (0x04)
#define BSP_COMPASS_REG_DATA_Z_MSB         (0x05)
#define BSP_COMPASS_REG_DATA_Z_LSB         (0x06)
#define BSP_COMPASS_REG_DATA_Y_MSB         (0x07)
#define BSP_COMPASS_REG_DATA_Y_LSB         (0x08)
#define BSP_COMPASS_REG_STATUS             (0x09)
#define BSP_COMPASS_REG_ID_A               (0x0A)
#define BSP_COMPASS_REG_ID_B               (0x0B)
#define BSP_COMPASS_REG_ID_C               (0x0C)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Compass data output rate
 */
typedef enum
{
  BSP_COMPASS_RATE_0_75HZ = 0x00,
  BSP_COMPASS_RATE_1_5HZ  = 0x04,
  BSP_COMPASS_RATE_3HZ    = 0x08,
  BSP_COMPASS_RATE_7_5HZ  = 0x0C,
  BSP_COMPASS_RATE_15HZ   = 0x10,  // Default
  BSP_COMPASS_RATE_30HZ   = 0x14,
  BSP_COMPASS_RATE_75HZ   = 0x18,
} bsp_compass_rate_t;

/**
 * @brief Compass gain settings
 */
typedef enum
{
  BSP_COMPASS_GAIN_0_88GA = 0x00,  // ±0.88 Ga
  BSP_COMPASS_GAIN_1_3GA  = 0x20,  // ±1.3 Ga (default)
  BSP_COMPASS_GAIN_1_9GA  = 0x40,  // ±1.9 Ga
  BSP_COMPASS_GAIN_2_5GA  = 0x60,  // ±2.5 Ga
  BSP_COMPASS_GAIN_4_0GA  = 0x80,  // ±4.0 Ga
  BSP_COMPASS_GAIN_4_7GA  = 0xA0,  // ±4.7 Ga
  BSP_COMPASS_GAIN_5_6GA  = 0xC0,  // ±5.6 Ga
  BSP_COMPASS_GAIN_8_1GA  = 0xE0,  // ±8.1 Ga
} bsp_compass_gain_t;

/**
 * @brief Compass operating mode
 */
typedef enum
{
  BSP_COMPASS_MODE_CONTINUOUS = 0x00,
  BSP_COMPASS_MODE_SINGLE     = 0x01,
  BSP_COMPASS_MODE_IDLE       = 0x02,
} bsp_compass_mode_t;

/**
 * @brief Compass sample averaging
 */
typedef enum
{
  BSP_COMPASS_AVG_1 = 0x00,
  BSP_COMPASS_AVG_2 = 0x20,
  BSP_COMPASS_AVG_4 = 0x40,
  BSP_COMPASS_AVG_8 = 0x60,  // Default
} bsp_compass_avg_t;

/**
 * @brief Cardinal directions
 */
typedef enum
{
  BSP_COMPASS_DIR_N = 0,
  BSP_COMPASS_DIR_NE,
  BSP_COMPASS_DIR_E,
  BSP_COMPASS_DIR_SE,
  BSP_COMPASS_DIR_S,
  BSP_COMPASS_DIR_SW,
  BSP_COMPASS_DIR_W,
  BSP_COMPASS_DIR_NW,
} bsp_compass_direction_t;

/**
 * @brief Compass raw data structure
 */
typedef struct
{
  int16_t raw_x;  // Raw X-axis value
  int16_t raw_y;  // Raw Y-axis value
  int16_t raw_z;  // Raw Z-axis value
} bsp_compass_raw_data_t;

/**
 * @brief Compass processed data structure
 */
typedef struct
{
  float                   heading_deg;    // Heading in degrees (0-360)
  float                   heading_rad;    // Heading in radians (-π to +π)
  float                   filtered_x;     // Filtered X-axis value
  float                   filtered_y;     // Filtered Y-axis value
  float                   filtered_z;     // Filtered Z-axis value
  bsp_compass_direction_t direction;      // Cardinal direction
  const char             *direction_str;  // Direction string ("N", "NE", etc.)
  size_t                  timestamp_ms;   // Timestamp of last reading
  bool                    is_valid;       // True if reading is valid
  bool                    is_overflow;    // True if overflow detected
} bsp_compass_data_t;

/**
 * @brief Compass callback function type
 */
typedef void (*bsp_compass_callback_t)(bsp_compass_data_t *data);

/**
 * @brief Compass raw data callback function type
 */
typedef void (*bsp_compass_raw_callback_t)(bsp_compass_raw_data_t *data);

/**
 * @brief Compass configuration structure
 */
typedef struct
{
  uint8_t            i2c_addr;        // I2C address (default 0x1E)
  uint8_t            sda_pin;         // I2C SDA pin
  uint8_t            scl_pin;         // I2C SCL pin
  size_t             i2c_clock;       // I2C clock frequency
  bsp_compass_rate_t data_rate;       // Data output rate
  bsp_compass_gain_t gain;            // Gain setting
  bsp_compass_mode_t mode;            // Operating mode
  bsp_compass_avg_t  averaging;       // Sample averaging
  float              ema_alpha;       // EMA filter coefficient (0.0 - 1.0)
  uint16_t           update_rate_ms;  // Update rate in milliseconds
} bsp_compass_config_t;

/**
 * @brief Compass context structure
 */
typedef struct
{
  bsp_compass_config_t       config;          // Sensor configuration
  bsp_compass_data_t         data;            // Processed sensor data
  bsp_compass_raw_data_t     raw_data;        // Raw sensor data
  bsp_compass_callback_t     callback;        // User callback for processed data
  bsp_compass_raw_callback_t raw_callback;    // User callback for raw data
  float                      ema_x;           // EMA filtered X
  float                      ema_y;           // EMA filtered Y
  float                      ema_z;           // EMA filtered Z
  size_t                     last_update_ms;  // Last update timestamp
  bool                       is_initialized;  // Initialization flag
  bool                       filter_ready;    // Filter initialized flag
} bsp_compass_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize compass with default configuration
 *
 * @param[in] ctx     Pointer to compass context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_init(bsp_compass_t *ctx);

/**
 * @brief Initialize compass with custom configuration
 *
 * @param[in] ctx     Pointer to compass context
 * @param[in] config  Pointer to configuration structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_init_with_config(bsp_compass_t *ctx, const bsp_compass_config_t *config);

/**
 * @brief Start the compass sensor
 *
 * @param[in] ctx     Pointer to compass context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_begin(bsp_compass_t *ctx);

/**
 * @brief Update sensor readings and process data
 *
 * @param[in] ctx     Pointer to compass context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_update(bsp_compass_t *ctx);

/**
 * @brief Read raw compass data
 *
 * @param[in]  ctx     Pointer to compass context
 * @param[out] data    Pointer to raw data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_read_raw(bsp_compass_t *ctx, bsp_compass_raw_data_t *data);

/**
 * @brief Get processed compass data
 *
 * @param[in]  ctx     Pointer to compass context
 * @param[out] data    Pointer to data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_get_data(bsp_compass_t *ctx, bsp_compass_data_t *data);

/**
 * @brief Get current heading in degrees (0-360)
 *
 * @param[in]  ctx         Pointer to compass context
 * @param[out] heading_deg Pointer to heading value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_get_heading_deg(bsp_compass_t *ctx, float *heading_deg);

/**
 * @brief Get current heading in radians
 *
 * @param[in]  ctx         Pointer to compass context
 * @param[out] heading_rad Pointer to heading value
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_get_heading_rad(bsp_compass_t *ctx, float *heading_rad);

/**
 * @brief Get cardinal direction
 *
 * @param[in]  ctx       Pointer to compass context
 * @param[out] direction Pointer to direction enum
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_get_direction(bsp_compass_t *ctx, bsp_compass_direction_t *direction);

/**
 * @brief Get cardinal direction as string
 *
 * @param[in]  ctx           Pointer to compass context
 * @param[out] direction_str Pointer to direction string pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_get_direction_str(bsp_compass_t *ctx, const char **direction_str);

/**
 * @brief Register callback for processed data
 *
 * @param[in] ctx      Pointer to compass context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_register_callback(bsp_compass_t *ctx, bsp_compass_callback_t callback);

/**
 * @brief Register callback for raw data
 *
 * @param[in] ctx      Pointer to compass context
 * @param[in] callback Callback function pointer
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_register_raw_callback(bsp_compass_t *ctx, bsp_compass_raw_callback_t callback);

/**
 * @brief Set EMA filter coefficient
 *
 * @param[in] ctx   Pointer to compass context
 * @param[in] alpha Filter coefficient (0.0 - 1.0, lower = smoother)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_set_filter_alpha(bsp_compass_t *ctx, float alpha);

/**
 * @brief Set compass gain
 *
 * @param[in] ctx  Pointer to compass context
 * @param[in] gain Gain setting
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_set_gain(bsp_compass_t *ctx, bsp_compass_gain_t gain);

/**
 * @brief Set data output rate
 *
 * @param[in] ctx  Pointer to compass context
 * @param[in] rate Data rate setting
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_set_rate(bsp_compass_t *ctx, bsp_compass_rate_t rate);

/**
 * @brief Reset EMA filter
 *
 * @param[in] ctx Pointer to compass context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_reset_filter(bsp_compass_t *ctx);

/**
 * @brief Reset I2C bus (recover from bus hang)
 *
 * @param[in] ctx Pointer to compass context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_reset_i2c(bsp_compass_t *ctx);

/**
 * @brief Scan I2C bus for devices (debug helper)
 *
 * @param[in] sda_pin SDA pin number
 * @param[in] scl_pin SCL pin number
 *
 * @return int8_t Found I2C address, or -1 if not found
 */
int8_t bsp_compass_scan_i2c(uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Check if compass is initialized
 *
 * @param[in] ctx Pointer to compass context
 *
 * @return true if initialized, false otherwise
 */
bool bsp_compass_is_initialized(bsp_compass_t *ctx);

/**
 * @brief Deinitialize compass and free resources
 *
 * @param[in] ctx Pointer to compass context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_compass_deinit(bsp_compass_t *ctx);

#endif /*End file _BSP_COMPASS_H_*/

/* End of file -------------------------------------------------------- */
