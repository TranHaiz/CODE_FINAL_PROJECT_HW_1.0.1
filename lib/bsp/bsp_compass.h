/**
 * @file       bsp_compass.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-03-08
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

/* Public defines ----------------------------------------------------- */
#define BSP_COMPASS_OVERFLOW_VALUE (-4096)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_COMPASS_RATE_0_75HZ = 0x00,
  BSP_COMPASS_RATE_1_5HZ  = 0x04,
  BSP_COMPASS_RATE_3HZ    = 0x08,
  BSP_COMPASS_RATE_7_5HZ  = 0x0C,
  BSP_COMPASS_RATE_15HZ   = 0x10,  // Default
  BSP_COMPASS_RATE_30HZ   = 0x14,
  BSP_COMPASS_RATE_75HZ   = 0x18,
} bsp_compass_sample_rate_t;

/**
 * @brief Compass gain settings (Config B bits [7:5])
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
 * @brief Compass sample averaging (Config A bits [6:5])
 */
typedef enum
{
  BSP_COMPASS_AVG_1 = 0x00,
  BSP_COMPASS_AVG_2 = 0x20,
  BSP_COMPASS_AVG_4 = 0x40,
  BSP_COMPASS_AVG_8 = 0x60,  // Default
} bsp_compass_avg_t;

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
 * @brief Compass configuration structure
 */
typedef struct
{
  bsp_compass_sample_rate_t data_rate;  // Data output rate
  bsp_compass_gain_t        gain;       // Gain setting
  bsp_compass_mode_t        mode;       // Operating mode
  bsp_compass_avg_t         averaging;  // Sample averaging
} bsp_compass_config_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize compass with default configuration
 *
 * @return status_function_t STATUS_OK on success
 */
status_function_t bsp_compass_init(void);

/**
 * @brief Configure compass settings
 *
 * @param[in] config Pointer to configuration structure
 *
 * @return status_function_t STATUS_OK on success
 */
status_function_t bsp_compass_config(const bsp_compass_config_t *config);

/**
 * @brief Read raw compass data from sensor
 *
 * @param[out] data Pointer to raw data structure
 *
 * @return status_function_t STATUS_OK on success, STATUS_ERROR on overflow or failure
 */
status_function_t bsp_compass_read_raw(bsp_compass_raw_data_t *data);

#endif /* _BSP_COMPASS_H_ */

/* End of file -------------------------------------------------------- */
