/**
 * @file       common_type.c
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-12
 * @author     Hai Tran
 *
 * @brief      Device information and configuration definitions
 *
 */

/* Includes ----------------------------------------------------------- */
#include "device_info.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
device_info_t g_device_info;

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void device_info_init(void)
{
  char buffer[MQTT_MAX_TOPIC_LEN];
  g_device_info.state = DEVICE_STATE_LOCKED;
  strncpy(g_device_info.device_name, DEFAULT_DEVICE_NAME, DEVICE_NAME_MAX_LEN);
  snprintf(g_device_info.mqtt_cmd_topic, sizeof(g_device_info.mqtt_cmd_topic), "%s/cmd", g_device_info.device_name);
  snprintf(g_device_info.mqtt_data_topic, sizeof(g_device_info.mqtt_data_topic), "%s/data", g_device_info.device_name);
}

/* Private definitions ----------------------------------------------- */

/* End of file ------------------------------------------------------- */
