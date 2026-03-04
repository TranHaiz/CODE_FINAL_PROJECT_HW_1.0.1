/**
 * @file       bsp_error.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-13
 * @author     Hai Tran
 *
 * @brief      BSP error handling definitions
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_error.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_error_handler(bsp_error_t error_code)
{
  switch (error_code)
  {
  case BSP_ERROR_SIM_INIT:
  case BSP_ERROR_SIM_GET_DATA_FIREBASE:
  case BSP_ERROR_SIM_SEND_DATA_FIREBASE:
  {
    bsp_sim_reset_http();
    break;
  }
  default: break;
  }
}
/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
