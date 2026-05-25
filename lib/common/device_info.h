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
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  DEVICE_STATE_IDLE = 0,
  DEVICE_STATE_LOCKED,
  DEVICE_STATE_ACTIVE,
  DEVICE_STATE_ERROR,
  DEVICE_STATE_PAUSED,
  DEVICE_STATE_NOTI,
  DEVICE_STATE_STOLEN,
  DEVICE_STATE_MAX
} device_state_t;
typedef enum
{
  DEVICE_DANGER_LEVEL_LOW = 0,
  DEVICE_DANGER_LEVEL_HIGH
} device_danger_level_t;

typedef esp_reset_reason_t device_reset_reason_t;
typedef struct
{
  uint8_t        err_count;
  uint8_t        device_id;
  device_state_t curr_state;
  device_state_t prev_state;
  float          total_km;
  char           serial_number[DEVICE_SERIAL_NUMBER_MAX_LEN];
} device_nvs_info_t;
typedef struct
{
  device_reset_reason_t last_reset_reason;
  char                  device_version[DEVICE_VERSION_LEN];
  char                  device_name[DEVICE_NAME_MAX_LEN];
  char                  mqtt_cmd_topic[MQTT_MAX_TOPIC_LEN];
  char                  last_mqtt_cmd_topic[MQTT_MAX_TOPIC_LEN];
  char                  mqtt_data_topic[MQTT_MAX_TOPIC_LEN];
  char                  mqtt_noti_topic[MQTT_MAX_TOPIC_LEN];
  char                  log_sd_path[DEVICE_LOG_SD_PATH_MAX_LEN];
  device_nvs_info_t     nvs_info;
  bool                  danger_noti_enabled;
  device_danger_level_t danger_level;
} device_info_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern RTC_DATA_ATTR device_info_t g_device_info;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize device information, read from flash, and set up logging
 * @return none
 */
void device_info_init(void);

/**
 * @brief Update device state and persist to flash
 * @param[in] new_state  New device state to update
 * @return none
 */
void device_info_update_state(device_state_t new_state);

/**
 * @brief Increment error count and persist to flash
 * @return none
 */
void device_info_inc_error_count(void);

/**
 * @brief Reset error count to zero and persist to flash
 * @return none
 */
void device_info_reset_error_count(void);

#endif /*End file _DEVICE_INFO_H_*/

/* End of file -------------------------------------------------------- */
