/**
 * @file       bsp_device.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-11
 * @author     Haq
 *
 * @brief      BSP device control definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_DEVICE_H_
#define _BSP_DEVICE_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "device_info.h"

#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Reboot the device
 */
void bsp_device_reboot(void);

/**
 * @brief Get the reason for the last reset
 * @return esp_reset_reason_t The reason for the last reset
 */
esp_reset_reason_t bsp_device_get_reset_reason(void);

/**
 * @brief Write RTC RAM
 * @param[in] p_info Pointer to device info structure to write to RTC RAM
 * @return None
 */
void bsp_device_rtc_write(device_nvs_info_t *p_info);

/**
 * @brief Read RTC RAM
 * @param[out] p_info Pointer to device info structure to read from RTC RAM
 * @return Status function
 */
status_function_t bsp_device_rtc_read(device_nvs_info_t *p_info);

/**
 * @brief Save device info to SD card (/info.json) atomically
 * @param[in] p_info Pointer to device info structure to persist
 * @return Status function
 */
status_function_t bsp_device_info_save(const device_nvs_info_t *p_info);

/**
 * @brief Load device info from SD card (/info.json)
 * @param[out] p_info Pointer to device info structure to fill
 * @return STATUS_OK if loaded, STATUS_ERROR if missing/corrupt
 */
status_function_t bsp_device_info_load(device_nvs_info_t *p_info);

/**
 * @brief Read legacy device info from flash (NVS). Migration path only.
 * @param[out] p_info Pointer to device info structure to read from flash
 * @return Status function
 */
status_function_t bsp_device_flash_read(device_nvs_info_t *p_info);

/**
 * @brief Erase legacy device info from flash (NVS). Migration path only.
 * @return Status function
 */
status_function_t bsp_device_flash_erase(void);

#endif /*End file _BSP_DEVICE_H_*/

/* End of file -------------------------------------------------------- */