/**
 * @file       bsp_temp_hum.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief    Temperature and humidity sensor
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_TEMP_HUM_H_
#define _BSP_TEMP_HUM_H_
/* Includes ----------------------------------------------------------- */
#include "Adafruit_SHT31.h"
#include "common_type.h"
#include "device_info.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief   Initialize the temperature and humidity sensor
 *
 * @return  Status function
 */
status_function_t bsp_temp_hum_init(void);

/**
 * @brief   Read temperature and humidity data from the sensor
 *
 * @param[in]   data  Pointer to a structure to store the read temperature and humidity data
 *
 * @return  Status function
 */
status_function_t bsp_temp_hum_read(temp_hum_data_t *data);

#endif /*End file _BSP_TEMP_HUM_H_*/

/* End of file -------------------------------------------------------- */
