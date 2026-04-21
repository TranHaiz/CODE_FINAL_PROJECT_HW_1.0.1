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
#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "log_service.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(device_info, LOG_LEVEL_DBG)

#define DEVICE_INIT_NEW_LOG_RETRIES    (3)
#define DEVICE_INIT_NEW_FOLDER_RETRIES (3)
#define DEVICE_LOG_FOLDER_PATH         "/logs"
#define DEVICE_OFF_LOG_FOLDER_PATH     "/buff"

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
  timeline_t        timeline;
  bsp_sdcard_file_t log_file;
  status_function_t ret = bsp_device_check_magic_number();
  bsp_rtc_get(&timeline);

  if (ret == STATUS_OK)
  {
    bsp_device_flash_read(&g_device_info.nvs_info);
    g_device_info.last_reset_reason = ESP_RST_UNKNOWN;
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

    g_device_info.last_reset_reason = bsp_device_get_reset_reason();
    switch (g_device_info.last_reset_reason)
    {
    case ESP_RST_EXT:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_SDIO:
    {
      g_device_info.nvs_info.curr_state = DEVICE_STATE_ERROR;
      break;
    }
    case ESP_RST_DEEPSLEEP:
    case ESP_RST_BROWNOUT:
    case ESP_RST_UNKNOWN:
    case ESP_RST_POWERON:
    case ESP_RST_SW:
    {
      if (g_device_info.nvs_info.last_state == DEVICE_STATE_ACTIVE)
      {
        g_device_info.nvs_info.curr_state = DEVICE_STATE_ACTIVE;
      }
      else
      {
        g_device_info.nvs_info.curr_state = DEVICE_STATE_LOCKED;
      }

      break;
    }
    default:
    {
      g_device_info.nvs_info.curr_state = DEVICE_STATE_ERROR;
      break;
    }
    }
  }

  snprintf(g_device_info.device_version, sizeof(g_device_info.device_version), "%d.%d.%d", FIRRMWARE_MAJOR_VERSION,
           FIRRMWARE_MINOR_VERSION, FIRRMWARE_PATCH_VERSION);
  snprintf(g_device_info.device_name, sizeof(g_device_info.device_name), "haq-trk-%03u",
           g_device_info.nvs_info.device_id);
  snprintf(g_device_info.mqtt_cmd_topic, sizeof(g_device_info.mqtt_cmd_topic), "%s/cmd", g_device_info.device_name);
  snprintf(g_device_info.mqtt_data_topic, sizeof(g_device_info.mqtt_data_topic), "%s/data", g_device_info.device_name);
  strncpy(g_device_info.last_mqtt_cmd_topic, g_device_info.mqtt_cmd_topic,
          sizeof(g_device_info.last_mqtt_cmd_topic) - 1);
  snprintf(g_device_info.mqtt_noti_topic, sizeof(g_device_info.mqtt_noti_topic), "%s/noti", g_device_info.device_name);

  snprintf(g_device_info.log_sd_path, sizeof(g_device_info.log_sd_path), "/logs/%d-%d-%d.log", timeline.date,
           timeline.month, timeline.year);

  if (bsp_sdcard_dir_exists(DEVICE_LOG_FOLDER_PATH) != STATUS_OK)
  {
    LOG_DBG("Log folder does not exist, creating: %s", DEVICE_LOG_FOLDER_PATH);
    for (int i = 0; i < DEVICE_INIT_NEW_FOLDER_RETRIES; i++)
    {
      if (bsp_sdcard_mkdir(DEVICE_LOG_FOLDER_PATH) != STATUS_OK)
      {
        LOG_WRN("Failed to create log folder: %s", DEVICE_LOG_FOLDER_PATH);
      }
      else
      {
        LOG_DBG("Log folder created successfully");
        break;
      }
      delay(100);
    }
  }

  if (bsp_sdcard_dir_exists(DEVICE_OFF_LOG_FOLDER_PATH) != STATUS_OK)
  {
    LOG_DBG("Log folder does not exist, creating: %s", DEVICE_OFF_LOG_FOLDER_PATH);
    for (int i = 0; i < DEVICE_INIT_NEW_FOLDER_RETRIES; i++)
    {
      if (bsp_sdcard_mkdir(DEVICE_OFF_LOG_FOLDER_PATH) != STATUS_OK)
      {
        LOG_WRN("Failed to create log folder: %s", DEVICE_OFF_LOG_FOLDER_PATH);
      }
      else
      {
        LOG_DBG("Log off folder created successfully");
        break;
      }
      delay(100);
    }
  }

  if (bsp_sdcard_file_exists(g_device_info.log_sd_path) == STATUS_OK)
  {
    LOG_DBG("Log file already exists for today, will append logs to it: %s", g_device_info.log_sd_path);
  }
  else
  {
    LOG_DBG("No log file for today, will create new log file: %s", g_device_info.log_sd_path);
    for (int i = 0; i < DEVICE_INIT_NEW_LOG_RETRIES; i++)
    {
      if (bsp_sdcard_open(g_device_info.log_sd_path, BSP_SDCARD_MODE_WRITE, &log_file) == STATUS_OK)
      {
        LOG_DBG("Log file created successfully");
        bsp_sdcard_close(&log_file);
        break;
      }
      LOG_WRN("Failed to create log file (attempt %d), retrying...", i + 1);
      delay(100);
    }
  }

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

void device_info_update_state(device_state_t new_state)
{
  g_device_info.nvs_info.last_state = g_device_info.nvs_info.curr_state;
  g_device_info.nvs_info.curr_state = new_state;
  bsp_device_flash_write(&g_device_info.nvs_info);
}

/* Private definitions ----------------------------------------------- */

/* End of file ------------------------------------------------------- */
