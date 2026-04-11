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

#include "bsp_device.h"
#include "log_service.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(device_info, LOG_LEVEL_DBG)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
device_info_t g_device_info;

// clang-format off
#define INFO(id, str) [id] = str
static const char *RESET_REASON_STR[ESP_RST_SDIO + 1] = { 
  /*RESET REASON            | DESCRIPTION */
  INFO(ESP_RST_UNKNOWN      , "Unknown"),
  INFO(ESP_RST_POWERON      , "Power-on"),
  INFO(ESP_RST_EXT          , "External pin"),
  INFO(ESP_RST_SW           , "Software reset"),
  INFO(ESP_RST_PANIC        , "Panic/Exception"),
  INFO(ESP_RST_INT_WDT      , "Interrupt Watchdog"),
  INFO(ESP_RST_TASK_WDT     , "Task Watchdog"),
  INFO(ESP_RST_WDT          , "Other Watchdog"),
  INFO(ESP_RST_DEEPSLEEP    , "Deep Sleep Wakeup"),
  INFO(ESP_RST_BROWNOUT     , "Brownout"),
  INFO(ESP_RST_SDIO         , "SDIO Reset") 
};
#undef INFO
// clang-format on

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void device_info_init(void)
{
  status_function_t ret = bsp_device_check_magic_number();

  if (ret == STATUS_OK)
  {
    bsp_device_flash_read(&g_device_info.nvs_info);
    LOG_INF("FLASH NVS data loaded successfully");
  }
  else
  {
    LOG_INF("No NVS data found, initializing with default values");
    // Initial default device info
    g_device_info.nvs_info.device_id = 0;
    snprintf(g_device_info.device_version, sizeof(g_device_info.device_version), "%d.%d.%d", FIRRMWARE_MAJOR_VERSION,
             FIRRMWARE_MINOR_VERSION, FIRRMWARE_PATCH_VERSION);
    snprintf(g_device_info.nvs_info.serial_number, sizeof(g_device_info.nvs_info.serial_number), "%032u",
             random(1, 0xFFFFFFFF));

    bsp_device_flash_write(&g_device_info.nvs_info);
  }

  g_device_info.state             = DEVICE_STATE_LOCKED;
  g_device_info.last_reset_reason = bsp_device_get_reset_reason();
  snprintf(g_device_info.device_version, sizeof(g_device_info.device_version), "%d.%d.%d", FIRRMWARE_MAJOR_VERSION,
           FIRRMWARE_MINOR_VERSION, FIRRMWARE_PATCH_VERSION);
  snprintf(g_device_info.device_name, sizeof(g_device_info.device_name), "haq-trk-%03u",
           g_device_info.nvs_info.device_id);
  snprintf(g_device_info.mqtt_cmd_topic, sizeof(g_device_info.mqtt_cmd_topic), "%s/cmd", g_device_info.device_name);
  snprintf(g_device_info.mqtt_data_topic, sizeof(g_device_info.mqtt_data_topic), "%s/data", g_device_info.device_name);

  LOG_INF("-- DEVICE INFO INITIALIZED ---");
  LOG_INF("Device ID: %u", g_device_info.nvs_info.device_id);
  LOG_INF("Device Version: %s", g_device_info.device_version);
  LOG_INF("Last Reset Reason: %s", RESET_REASON_STR[g_device_info.last_reset_reason]);
  LOG_INF("Serial Number: %s", g_device_info.nvs_info.serial_number);
  LOG_INF("Device Name: %s", g_device_info.device_name);
  LOG_INF("MQTT Cmd Topic: %s", g_device_info.mqtt_cmd_topic);
  LOG_INF("MQTT Data Topic: %s", g_device_info.mqtt_data_topic);
  LOG_INF("--------------------------------");
}

/* Private definitions ----------------------------------------------- */

/* End of file ------------------------------------------------------- */
