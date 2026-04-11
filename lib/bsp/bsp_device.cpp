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

#include "esp_system.h"

#include <Preferences.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
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

status_function_t bsp_device_flash_write(device_nvs_info_t *p_info)
{
  if (p_info == NULL)
  {
    return STATUS_ERROR;
  }

  if (!s_prefs.begin(DEVICE_NVS_NAMESPACE, false))  // false = read-write
  {
    return STATUS_ERROR;
  }

  size_t written = s_prefs.putBytes(DEVICE_NVS_KEY_INFO, p_info, sizeof(device_nvs_info_t));
  s_prefs.end();

  if (written != sizeof(device_nvs_info_t))
  {
    return STATUS_ERROR;
  }

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

status_function_t bsp_device_check_magic_number(void)
{
  if (!s_prefs.begin(DEVICE_NVS_NAMESPACE, false))
  {
    return STATUS_ERROR;
  }

  uint32_t magic = s_prefs.getUInt(DEVICE_NVS_KEY_MAGIC, 0);

  if (magic == DEVICE_MAGIC_NUMBER)
  {
    s_prefs.end();
    return STATUS_OK;
  }

  s_prefs.putUInt(DEVICE_NVS_KEY_MAGIC, DEVICE_MAGIC_NUMBER);
  s_prefs.end();

  return STATUS_ERROR;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */