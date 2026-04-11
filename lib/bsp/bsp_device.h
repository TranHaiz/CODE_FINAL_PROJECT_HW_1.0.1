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
 * @brief Write device info to flash (NVS)
 * @param[in] p_info Pointer to device info structure to write to flash
 * @return Status function
 */
status_function_t bsp_device_flash_write(device_nvs_info_t *p_info);

/**
 * @brief Read device info from flash (NVS)
 * @param[out] p_info Pointer to device info structure to read from flash
 * @return Status function
 */
status_function_t bsp_device_flash_read(device_nvs_info_t *p_info);

/**
 * @brief Erase device info from flash (NVS)
 * @return Status function
 */
status_function_t bsp_device_flash_erase(void);

/**
 * @brief Check if device info in flash (NVS) is initialized
 * @return Status function
 */
status_function_t bsp_device_check_magic_number(void);

#endif /*End file _BSP_DEVICE_H_*/

/* End of file -------------------------------------------------------- */