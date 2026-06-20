/**
 * @file       bsp_device.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-11
 * @author     Haq
 *
 * @brief      BSP device control definitions
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_device.h"

#include "bsp_sdcard.h"
#include "device_config.h"
#include "esp_system.h"
#include "log_service.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_device, LOG_LEVEL_BSP_DEVICE);

#define DEVICE_INFO_JSON_BUF_SIZE (256)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */

// RTC_DATA_ATTR: stored in RTC slow memory, retained across soft reset and deep sleep
RTC_DATA_ATTR static device_nvs_info_t rtc_device_info;
RTC_DATA_ATTR static bool              is_rtc_data_valid = false;

/* Private variables -------------------------------------------------- */
static Preferences s_prefs;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

void bsp_device_reboot(void)
{
  esp_restart();
}

esp_reset_reason_t bsp_device_get_reset_reason(void)
{
  return esp_reset_reason();
}

void bsp_device_rtc_write(device_nvs_info_t *p_info)
{
  if (p_info == NULL)
  {
    return;
  }

  memcpy(&rtc_device_info, p_info, sizeof(device_nvs_info_t));
  is_rtc_data_valid = true;
}

status_function_t bsp_device_rtc_read(device_nvs_info_t *p_info)
{
  if (p_info == NULL)
  {
    return STATUS_ERROR;
  }

  if (!is_rtc_data_valid)
  {
    return STATUS_ERROR;
  }

  memcpy(p_info, &rtc_device_info, sizeof(device_nvs_info_t));

  return STATUS_OK;
}

status_function_t bsp_device_info_save(const device_nvs_info_t *p_info)
{
  if (p_info == NULL)
  {
    return STATUS_ERROR;
  }

  JsonDocument doc;
  doc["device_id"]     = p_info->device_id;
  doc["serial_number"] = p_info->serial_number;
  doc["curr_state"]    = (int) p_info->curr_state;
  doc["prev_state"]    = (int) p_info->prev_state;
  doc["err_count"]     = p_info->err_count;
  doc["total_km"]      = p_info->total_km;

  char   buf[DEVICE_INFO_JSON_BUF_SIZE];
  size_t len = serializeJson(doc, buf, sizeof(buf));
  if (len == 0)
  {
    LOG_ERR("Failed to serialize device info");
    return STATUS_ERROR;
  }

  bsp_sdcard_file_t file;
  if (bsp_sdcard_open(SD_INFO_TMP_PATH, BSP_SDCARD_MODE_WRITE, &file) != STATUS_OK)
  {
    LOG_ERR("Failed to open %s", SD_INFO_TMP_PATH);
    return STATUS_ERROR;
  }
  size_t            written = 0;
  status_function_t ret     = bsp_sdcard_write(&file, (const uint8_t *) buf, len, &written);
  bsp_sdcard_close(&file);

  if (ret != STATUS_OK || written != len)
  {
    LOG_ERR("Failed to write device info (%u/%u)", (unsigned) written, (unsigned) len);
    bsp_sdcard_delete(SD_INFO_TMP_PATH);
    return STATUS_ERROR;
  }

  if (bsp_sdcard_file_exists(SD_INFO_PATH) == STATUS_OK)
  {
    bsp_sdcard_delete(SD_INFO_PATH);
  }
  if (bsp_sdcard_rename(SD_INFO_TMP_PATH, SD_INFO_PATH) != STATUS_OK)
  {
    LOG_ERR("Failed to commit %s", SD_INFO_PATH);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_device_info_load(device_nvs_info_t *p_info)
{
  if (p_info == NULL)
  {
    return STATUS_ERROR;
  }

  bsp_sdcard_file_t file;
  if (bsp_sdcard_open(SD_INFO_PATH, BSP_SDCARD_MODE_READ, &file) != STATUS_OK)
  {
    return STATUS_ERROR;  // no info.json yet (first boot)
  }

  char              buf[DEVICE_INFO_JSON_BUF_SIZE];
  size_t            read_len = 0;
  status_function_t ret      = bsp_sdcard_read(&file, (uint8_t *) buf, sizeof(buf) - 1, &read_len);
  bsp_sdcard_close(&file);

  if (ret != STATUS_OK || read_len == 0)
  {
    LOG_ERR("Failed to read %s", SD_INFO_PATH);
    return STATUS_ERROR;
  }
  buf[read_len] = '\0';

  JsonDocument         doc;
  DeserializationError err = deserializeJson(doc, buf);
  if (err)
  {
    LOG_ERR("Corrupt info.json: %s", err.c_str());
    return STATUS_ERROR;
  }

  p_info->device_id  = doc["device_id"] | 0;
  p_info->curr_state = (device_state_t) (doc["curr_state"] | (int) DEVICE_STATE_LOCKED);
  p_info->prev_state = (device_state_t) (doc["prev_state"] | (int) DEVICE_STATE_LOCKED);
  p_info->err_count  = doc["err_count"] | 0;
  p_info->total_km   = doc["total_km"] | 0.0f;

  const char *serial = doc["serial_number"] | "";
  strncpy(p_info->serial_number, serial, sizeof(p_info->serial_number) - 1);
  p_info->serial_number[sizeof(p_info->serial_number) - 1] = '\0';

  return STATUS_OK;
}

status_function_t bsp_device_flash_read(device_nvs_info_t *p_info)
{
  if (p_info == NULL)
  {
    return STATUS_ERROR;
  }

  if (!s_prefs.begin(DEVICE_NVS_NAMESPACE, true))  // true = read-only
  {
    return STATUS_ERROR;
  }

  size_t len = s_prefs.getBytesLength(DEVICE_NVS_KEY_INFO);
  if (len != sizeof(device_nvs_info_t))
  {
    s_prefs.end();
    return STATUS_ERROR;
  }

  s_prefs.getBytes(DEVICE_NVS_KEY_INFO, p_info, sizeof(device_nvs_info_t));
  s_prefs.end();

  return STATUS_OK;
}

status_function_t bsp_device_flash_erase(void)
{
  if (!s_prefs.begin(DEVICE_NVS_NAMESPACE, false))
  {
    return STATUS_ERROR;
  }

  bool ok = s_prefs.clear();
  s_prefs.end();

  return ok ? STATUS_OK : STATUS_ERROR;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */