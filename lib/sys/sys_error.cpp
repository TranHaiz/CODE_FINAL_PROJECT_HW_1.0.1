/**
 * @file       sys_error.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      System error manager implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_error.h"

#include "bsp_device.h"
#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_ble.h"
#include "sys_led.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_error, LOG_LEVEL_SYS_ERROR);

#define SYS_ERROT_UPDATE_ERROR_MESSSAGE_MS (1000)
#define SYS_ERROR_MAX_MESSAGE              ("System Error: Device is in error state. Please check the device and restart.")

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  size_t last_error_send_time_ms;
} sys_error_handler_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sys_error_handler_t g_sys_error_handler;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void sys_error_init(void)
{
  memset(&g_sys_error_handler, 0, sizeof(g_sys_error_handler));
  bsp_device_flash_read(&g_device_info.nvs_info);
  snprintf(g_device_info.device_name, sizeof(g_device_info.device_name), "haq-trk-%03u",
           g_device_info.nvs_info.device_id);
  sys_ble_init();
  sys_led_init();
  sys_ble_send((const uint8_t *) SYS_ERROR_MAX_MESSAGE, strlen(SYS_ERROR_MAX_MESSAGE));
  g_sys_error_handler.last_error_send_time_ms = OS_GET_TICK();
}

void sys_error_process(void)
{
  size_t now_ms = OS_GET_TICK();
  if ((now_ms - g_sys_error_handler.last_error_send_time_ms >= SYS_ERROT_UPDATE_ERROR_MESSSAGE_MS)
      && sys_ble_is_connected())
  {
    sys_ble_send((const uint8_t *) SYS_ERROR_MAX_MESSAGE, strlen(SYS_ERROR_MAX_MESSAGE));
    g_sys_error_handler.last_error_send_time_ms = now_ms;
  }
  sys_ble_process();
  sys_led_process();
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */