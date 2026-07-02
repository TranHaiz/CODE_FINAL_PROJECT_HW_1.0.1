/**
 * @file       bsp_error.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-13
 * @author     Hai Tran
 *
 * @brief      BSP error handling definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_ERROR_H_
#define _BSP_ERROR_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
#define BSP_ERROR_MAX_COUNT (4)  // Consecutive fatal errors before entering error mode

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Handle BSP errors based on error code
 * @param[in] error_code The error code to handle
 * @return None
 */
void bsp_error_handler(device_error_t error_code);

/**
 * @brief Enter error mode
 * @param[in] err_cnt The current error count
 * @return None
 */
void bsp_error_check(void);

#endif /*End file _BSP_ERROR_H_*/

/* End of file -------------------------------------------------------- */
