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

#include "bsp_batt.h"
#include "bsp_device.h"
#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "bsp_servo.h"
#include "log_service.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(device_info, LOG_LEVEL_DEVICE_INFO)

#define DEVICE_INIT_NEW_LOG_RETRIES    (3)
#define DEVICE_INIT_NEW_FOLDER_RETRIES (3)
#define DEVICE_LOG_FOLDER_PATH         SD_LOG_DIR   // see SD layout in device_config.h
#define DEVICE_OFF_LOG_FOLDER_PATH     SD_BUFF_DIR

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
RTC_DATA_ATTR device_info_t g_device_info;

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
  bsp_rtc_get(&timeline);

  if (bsp_device_info_load(&g_device_info.nvs_info) == STATUS_OK)
  {
    g_device_info.last_reset_reason = ESP_RST_UNKNOWN;
    LOG_INF("Device info loaded from SD (info.json)");
  }
  else
  {
    // No info.json: migrate legacy NVS once if present, otherwise start fresh,
    // then create info.json on SD and wipe NVS so flash is never written again.
    if (bsp_device_flash_read(&g_device_info.nvs_info) == STATUS_OK)
    {
      LOG_INF("Migrating device info from NVS to SD");
    }
    else
    {
      LOG_INF("No saved device info, initializing with default values");
      memset(&g_device_info.nvs_info, 0, sizeof(g_device_info.nvs_info));
      g_device_info.nvs_info.device_id = 0;
      snprintf(g_device_info.nvs_info.serial_number, sizeof(g_device_info.nvs_info.serial_number), "%032u",
               random(1, 0xFFFFFFFF));
    }

    if (bsp_device_info_save(&g_device_info.nvs_info) == STATUS_OK)
    {
      bsp_device_flash_erase();
    }
    else
    {
      LOG_WRN("Failed to persist info.json; keeping NVS for next boot");
    }
  }

  g_device_info.last_reset_reason   = bsp_device_get_reset_reason();
  g_device_info.danger_noti_enabled = true;                     // Default enabled
  g_device_info.danger_level        = DEVICE_DANGER_LEVEL_LOW;  // Default danger level
  g_device_info.servo_sync_pending  = false;
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE)
  {
    device_info_update_state(DEVICE_STATE_LOCKED);
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

  snprintf(g_device_info.log_sd_path, sizeof(g_device_info.log_sd_path), SD_LOG_DIR "/%d-%d-%d.log", timeline.date,
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
  LOG_INF("Error Count: %d", g_device_info.nvs_info.err_count);
  LOG_INF("--------------------------------");
}

// Power is safe to actuate the servo unless the battery is critically low. When
// the monitor isn't up yet (early boot) only a prior brownout is treated as unsafe.
static bool device_servo_actuation_safe(void)
{
  if (!bsp_batt_is_initialized())
  {
    return (g_device_info.last_reset_reason != ESP_RST_BROWNOUT);
  }

  float   sum   = 0.0f;
  uint8_t count = 0;
  for (uint8_t i = 0; i < SERVO_BATT_SAFE_SAMPLES; i++)
  {
    float v = bsp_batt_read_voltage_mv();
    if (v > 0.0f)
    {
      sum += v;
      count++;
    }
  }
  if (count == 0)
  {
    return false;
  }
  float avg  = sum / count;
  bool  safe = (avg >= SERVO_BATT_SAFE_MV);
  LOG_DBG("Servo guard: V_avg=%.0fmV thr=%.0f -> %s", avg, (float) SERVO_BATT_SAFE_MV, safe ? "SAFE" : "DEFER");
  return safe;
}

void device_info_update_state(device_state_t new_state)
{
  g_device_info.nvs_info.prev_state = g_device_info.nvs_info.curr_state;
  g_device_info.nvs_info.curr_state = new_state;
  bsp_device_info_save(&g_device_info.nvs_info);

  device_info_apply_lock_state();
}

void device_info_apply_lock_state(void)
{
  // Defer when power is unsafe: the mechanism self-holds, so the lock stays put
  // until device_info_apply_lock_state() is retried once the battery recovers.
  if (!device_servo_actuation_safe())
  {
    g_device_info.servo_sync_pending = true;
    LOG_WRN("Servo actuation deferred (battery guard)");
    return;
  }

  // Physical lock follows device state: unlocked only while actively riding
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_ACTIVE)
  {
    LOG_DBG("Servo -> UNLOCK (state=%d)", g_device_info.nvs_info.curr_state);
    bsp_servo_unlock();
  }
  else
  {
    LOG_DBG("Servo -> LOCK (state=%d)", g_device_info.nvs_info.curr_state);
    bsp_servo_lock();
  }
  g_device_info.servo_sync_pending = false;
}

void device_info_inc_error_count(void)
{
  g_device_info.nvs_info.err_count++;
  bsp_device_info_save(&g_device_info.nvs_info);
}

void device_info_reset_error_count(void)
{
  g_device_info.nvs_info.err_count = 0;
  bsp_device_info_save(&g_device_info.nvs_info);
}

/* Private definitions ----------------------------------------------- */

/* End of file ------------------------------------------------------- */
