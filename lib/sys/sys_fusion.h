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
} sys_fusion_data_t;

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

#endif /*End file _SYS_FUSION_H_*/

/* End of file -------------------------------------------------------- */
