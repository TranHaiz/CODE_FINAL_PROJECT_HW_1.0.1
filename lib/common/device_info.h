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
#define DEVICE_NORMAL_MODE       (0)
#define DEVICE_FUSION_DEBUG_MODE (1)

/**
 * @brief  Server configuration
 */
#define CONFIG_FIREBASE_SERVER   (false)
#define CONFIG_MQTT_SERVER       (true)

/**
 * @brief  Screen rotation configuration
 */
#define SCREEN_ROTATION_0        (0)
#define SCREEN_ROTATION_90       (0)
#define SCREEN_ROTATION_180      (0)
#define SCREEN_ROTATION_270      (1)

#define DEFAULT_DEVICE_NAME      "haq-trk-000"
#define DEVICE_NAME_MAX_LEN      (32)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  DEVICE_STATE_IDLE = 0,
  DEVICE_STATE_LOCKED,
  DEVICE_STATE_ACTIVE,
  DEVICE_STATE_ERROR,
  DEVICE_STATE_MAX
} device_state_t;
typedef struct
{
  device_state_t state;
  char           device_name[DEVICE_NAME_MAX_LEN];
} device_info_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern device_info_t g_device_info;

/* Public function prototypes ----------------------------------------- */
void device_info_init(void);

#endif /*End file _DEVICE_INFO_H_*/

/* End of file -------------------------------------------------------- */
