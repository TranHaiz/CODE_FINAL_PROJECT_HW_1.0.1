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
#include "bsp_sim.h"
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_ERROR_NONE = 0,
  BSP_ERROR_SD_INIT,
  BSP_ERROR_SD_MOUNT,
  BSP_ERROR_SD_MKDIR,
  BSP_ERROR_SD_OPEN_FILE,
  BSP_ERROR_SIM_INIT,
  BSP_ERROR_SIM_SEND_DATA_FIREBASE,
  BSP_ERROR_SIM_GET_DATA_FIREBASE,
  BSP_ERROR_MAX
} bsp_error_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void bsp_error_handler(bsp_error_t error_code);

#endif /*End file _BSP_ERROR_H_*/

/* End of file -------------------------------------------------------- */
