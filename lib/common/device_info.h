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
#include "esp_system.h"

/* Public defines ----------------------------------------------------- */
#define FIRRMWARE_MAJOR_VERSION      (1)
#define FIRRMWARE_MINOR_VERSION      (0)
#define FIRRMWARE_PATCH_VERSION      (0)

#define DEVICE_NVS_NAMESPACE         "bsp_device"
#define DEVICE_NVS_KEY_INFO          "dev_info"
#define DEVICE_NVS_KEY_MAGIC         "dev_magic"
#define DEVICE_VERSION_LEN           (6)   // example: 1.0.0 (5 chars + 1 null terminator)
#define DEVICE_SERIAL_NUMBER_MAX_LEN (33)  // 32 chars + 1 null terminator
#define DEVICE_MAGIC_NUMBER          (0xDEADBEEF)

/**
 * @brief  Device operation modes
 * @note 0 - deactivated
 *       1 - normal mode
 */

#define DEVICE_NORMAL_MODE           (0)
#define DEVICE_FUSION_DEBUG_MODE     (1)

/**
 * @brief  Server configuration
 */
#define CONFIG_FIREBASE_SERVER       (false)
#define CONFIG_MQTT_SERVER           (true)

#define MQTT_MAX_TOPIC_LEN           (64)

#define SCREEN_SKIP_LOCK_SCREEN      (1)
#define SCREEN_ROTATION_0            (0)
#define SCREEN_ROTATION_90           (0)
#define SCREEN_ROTATION_180          (0)
#define SCREEN_ROTATION_270          (1)

#define DEFAULT_DEVICE_NAME          "haq-trk-000"
#define DEVICE_NAME_MAX_LEN          (32)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  DEVICE_STATE_IDLE = 0,
  DEVICE_STATE_LOCKED,
  DEVICE_STATE_ACTIVE,
  DEVICE_STATE_ERROR,
  DEVICE_STATE_MAX
} device_state_t;
typedef esp_reset_reason_t device_reset_reason_t;
typedef struct
{
  uint8_t device_id;
  char    serial_number[DEVICE_SERIAL_NUMBER_MAX_LEN];
} device_nvs_info_t;
typedef struct
{
  device_state_t        state;
  device_reset_reason_t last_reset_reason;
  char                  device_version[DEVICE_VERSION_LEN];
  char                  device_name[DEVICE_NAME_MAX_LEN];
  char                  mqtt_cmd_topic[MQTT_MAX_TOPIC_LEN];
  char                  mqtt_data_topic[MQTT_MAX_TOPIC_LEN];
  device_nvs_info_t     nvs_info;
} device_info_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern device_info_t g_device_info;

/* Public function prototypes ----------------------------------------- */
void device_info_init(void);

#endif /*End file _DEVICE_INFO_H_*/

/* End of file -------------------------------------------------------- */
