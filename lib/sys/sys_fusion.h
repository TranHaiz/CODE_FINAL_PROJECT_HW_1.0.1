/**
 * @file       sys_fusion.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-25
 * @author     Hai Tran
 *
 * @brief      Sensor fusion implementation
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_FUSION_H_
#define _SYS_FUSION_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
#if (DEVICE_FUSION_DEBUG_MODE == 1)
typedef struct
{
  // Accelerometer (g)
  float acc_raw_x;
  float acc_raw_y;
  float acc_raw_z;
  float acc_filter_x;
  float acc_filter_y;
  float acc_filter_z;

  // Gyroscope (dps)
  float gyro_raw_x;
  float gyro_raw_y;
  float gyro_raw_z;
  float gyro_filter_x;
  float gyro_filter_y;
  float gyro_filter_z;

  // Compass (raw counts / EMA-filtered counts)
  float compass_raw_x;
  float compass_raw_y;
  float compass_raw_z;
  float compass_filter_x;
  float compass_filter_y;
  float compass_filter_z;

  // Velocity components (m/s)
  float v_ins;
  float v_gps;

  // Distance accumulators (m)
  float distance_ins;  // INS distance since last GPS update
  float distance_gps;  // Last GPS step distance (haversine)
} sys_fusion_debug_data_t;
#endif /* DEVICE_FUSION_DEBUG_MODE */

/**
 * @brief Fusion output data structure
 */
typedef struct
{
  float               velocity_ms;
  float               velocity_kmh;
  float               distance_m;
  float               heading_deg;
  const char         *direction_str;
  gps_position_type_t gps_position;
#if (DEVICE_FUSION_DEBUG_MODE == 1)
  sys_fusion_debug_data_t debug;
#endif
} sys_fusion_data_t;

/**
 * @brief Detect dangerous motion patterns.
 */
typedef enum
{
  SYS_FUSION_DANGER_MOTION_NONE = 0,
  SYS_FUSION_DANGER_MOTION_TILT,
  SYS_FUSION_DANGER_MOTION_MOVING,
  SYS_FUSION_DANGER_MOTION_VIBRATION
} sys_fusion_danger_motion_flag_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize sensor fusion module (ACC, GPS, Compass)
 *
 * @return none
 */
void sys_fusion_init(void);

/**
 * @brief Process sensor fusion, poll in thread loop
 *
 * @param[out] data Pointer to fusion output data structure
 *
 * @return status_function_t Status of operation
 */
status_function_t sys_fusion_process(sys_fusion_data_t *data);

/**
 * @brief Detect dangerous motion patterns (e.g. fall, collision) using accelerometer data
 * @param[out] out_flags Optional pointer to receive detailed flags for detected motion types
 * @return none
 */
void sys_fusion_detect_danger_motion(sys_fusion_danger_motion_flag_t *out_flags);

#endif /*End file _SYS_FUSION_H_*/

/* End of file -------------------------------------------------------- */
