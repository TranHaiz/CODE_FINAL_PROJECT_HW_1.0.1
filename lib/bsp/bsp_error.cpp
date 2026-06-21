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

#include "bsp_device.h"
#include "bsp_sim.h"
#include "device_info.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_error_handler(bsp_error_t error_code)
{
  g_device_info.last_error_code = (uint8_t) error_code;  // RTC-persisted so error mode can report it
  device_info_inc_error_count();

  switch (error_code)
  {
  case BSP_ERROR_SD_INIT:
  case BSP_ERROR_SD_MOUNT:
  case BSP_ERROR_SD_MKDIR:
  case BSP_ERROR_SD_OPEN_FILE:
  case BSP_ERROR_DISPLAY_INIT:
  {
    bsp_device_reboot();
    break;
  }
#if (CONFIG_FIREBASE_SERVER)
  case BSP_ERROR_SIM_GET_DATA_FIREBASE:
  case BSP_ERROR_SIM_SEND_DATA_FIREBASE:
  {
    bsp_sim_reset_http();
    break;
  }
#endif
  default: break;
  }
}

void bsp_error_check(void)
{
  if ((g_device_info.nvs_info.err_count == BSP_ERROR_MAX_COUNT)
      && (g_device_info.nvs_info.curr_state != DEVICE_STATE_ERROR))
  {
    device_info_reset_error_count();
    g_device_info.nvs_info.prev_state = g_device_info.nvs_info.curr_state;
    g_device_info.nvs_info.curr_state = DEVICE_STATE_ERROR;
    device_info_apply_lock_state();
    bsp_device_reboot();
  }
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
