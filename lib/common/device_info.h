/**
 * @file       device_info.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Common type definitions used across the project
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _DEVICE_INFO_H_
#define _DEVICE_INFO_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "device_config.h"

/* Public defines ----------------------------------------------------- */
// #define CONFIG_SD_DEBUG_MODE
#define CONFIG_FIREBASE_SERVER (false)
#define CONFIG_MQTT_SERVER     (true)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  MODEM_USB = 0,
  MODEM_BLE,
  MODEM_WIFI,
  MODEM_MAX
} modem_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif /*End file _DEVICE_INFO_H_*/

/* End of file -------------------------------------------------------- */
