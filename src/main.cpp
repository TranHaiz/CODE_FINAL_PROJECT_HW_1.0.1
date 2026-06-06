/**
 * @file       main.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-28
 * @author     Hai Tran
 *
 * @brief     Main entry point for the project
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_buzzer.h"
#include "bsp_error.h"
#include "bsp_io.h"
#include "bsp_led.h"
#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "bsp_sim.h"
#include "common_type.h"
#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_button.h"
#include "sys_cmd.h"
#include "sys_cmd_usb.h"
#include "sys_error.h"
#include "sys_fusion_log.h"
#include "sys_input.h"
#include "sys_led.h"
#include "sys_log.h"
#include "sys_manager.h"
#include "sys_network.h"
#include "sys_network_adapter_ble.h"
#include "sys_network_adapter_lte.h"
#include "sys_ui.h"
#include "sys_ui_simple.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(main, LOG_LEVEL_MAIN)

#define SYS_INPUT_UPDATE_RATE_MS  (5)
#define SYS_UI_UPDATE_RATE_MS     (10)
#define SYS_LOG_UPDATE_RATE_MS    (500)
#define SYS_MISC_UPDATE_RATE_MS   (100)
#define SYS_BUTTON_UPDATE_RATE_MS (10)
#define SYS_ERROR_UPDATE_RATE_MS  (100)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
status_function_t g_ret = STATUS_ERROR;
sys_input_data_t  g_input_data;

#if (DEVICE_INPUT_ENABLED)
OS_THREAD_DECLARE(sys_input_thread, tskIDLE_PRIORITY + 3, 4096);
#endif

#if (DEVICE_NETWORK_ENABLED)
OS_THREAD_DECLARE(sys_network_thread, tskIDLE_PRIORITY + 3, 6144);
OS_THREAD_DECLARE(sys_network_lte_thread, tskIDLE_PRIORITY + 2, 8192);
#endif

#if (DEVICE_UI_ENABLED)
OS_THREAD_DECLARE(sys_ui_thread, tskIDLE_PRIORITY + 4, 16384);
#endif

OS_THREAD_DECLARE(sys_log_thread, tskIDLE_PRIORITY + 1, 4096);
OS_THREAD_DECLARE(sys_misc_thread, tskIDLE_PRIORITY + 4, 4096);
OS_THREAD_DECLARE(sys_cmd_thread, tskIDLE_PRIORITY + 5, 4096);
OS_THREAD_DECLARE(sys_cmd_usb_thread, tskIDLE_PRIORITY + 5, 4096);
OS_THREAD_DECLARE(sys_manager_thread, tskIDLE_PRIORITY + 5, 4096);
OS_THREAD_DECLARE(sys_button_thread, tskIDLE_PRIORITY + 4, 4096);
OS_THREAD_DECLARE(sys_error_thread, tskIDLE_PRIORITY + 5, 8192);
OS_THREAD_DECLARE(sys_ble_thread, tskIDLE_PRIORITY + 3, 8192);

/* Private function prototypes ---------------------------------------- */
void sys_input_thread_func(void *param);
void sys_ui_thread_func(void *param);
void sys_log_thread_func(void *param);
void sys_cmd_thread_func(void *param);
void sys_cmd_usb_thread_func(void *param);
void sys_manager_thread_func(void *param);
void sys_misc_thread_func(void *param);
void sys_button_thread_func(void *param);
void sys_error_thread_func(void *param);
void sys_ble_thread_func(void *param);

void kill_all_threads(void);

/* Function definitions ----------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  bsp_error_check();
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_ERROR)
  {
    log_service_set_timestamp(LOG_TIMESTAMP_EPOCH);
    OS_THREAD_CREATE(sys_error_thread, sys_error_thread_func);
    return;  // Skip normal initialization if in error state
  }
  log_service_set_timestamp(LOG_TIMESTAMP_RTC);
  bsp_rtc_init();
  bsp_sdcard_init();
  delay(1000);
  device_info_init();
  sys_network_adapter_ble_init();
  sys_network_init();
  delay(1000);
  OS_THREAD_CREATE(sys_cmd_thread, sys_cmd_thread_func);
  OS_THREAD_CREATE(sys_ble_thread, sys_ble_thread_func);
  OS_THREAD_CREATE(sys_cmd_usb_thread, sys_cmd_usb_thread_func);

#if (DEVICE_INPUT_ENABLED)
  OS_THREAD_CREATE(sys_input_thread, sys_input_thread_func);
#endif

#if (DEVICE_NETWORK_ENABLED && (DEVICE_FUSION_TUNING_MODE_ENABLED != 1))
  OS_THREAD_CREATE(sys_network_thread, sys_network_task);
  OS_THREAD_CREATE(sys_network_lte_thread, sys_network_adapter_lte_task);
#endif

#if (DEVICE_UI_ENABLED)
  OS_THREAD_CREATE(sys_ui_thread, sys_ui_thread_func);
#endif

  OS_THREAD_CREATE(sys_log_thread, sys_log_thread_func);
  OS_THREAD_CREATE(sys_misc_thread, sys_misc_thread_func);
  OS_THREAD_CREATE(sys_manager_thread, sys_manager_thread_func);
  OS_THREAD_CREATE(sys_button_thread, sys_button_thread_func);
}

void loop()
{
  OS_DELAY_MS(15000);  // Wait for threads to initialize before checking for errors
  bsp_error_check();
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_ERROR)
  {
    kill_all_threads();
    log_service_set_timestamp(LOG_TIMESTAMP_EPOCH);
    OS_THREAD_CREATE(sys_error_thread, sys_error_thread_func);
  }
  OS_DELAY_MS(OS_MAX_DELAY);
}

/* Private definitions ----------------------------------------------- */
void sys_input_thread_func(void *param)
{
  sys_input_init();

  while (true)
  {
    // Process sensor data
    g_ret = sys_input_process();
    if (g_ret != STATUS_OK)
    {
      // Do nothing, just wait for next cycle to retry
    }
    else
    {
      // Get current data
      if (sys_input_get_data(&g_input_data) == STATUS_OK)
      {
        is_data_network_ready = true;
      }
    }

    OS_DELAY_MS(SYS_INPUT_UPDATE_RATE_MS);
  }
}

void sys_ui_thread_func(void *param)
{
  sys_ui_init();

  while (true)
  {
    sys_ui_process();
    OS_DELAY_MS(SYS_UI_UPDATE_RATE_MS);
  }
}

void sys_log_thread_func(void *param)
{
  sys_log_init();
#if (DEVICE_FUSION_DEBUG_LOG_ENABLED == 1)
  sys_fusion_log_init();
#endif

  while (true)
  {
    sys_log_process();
#if (DEVICE_FUSION_DEBUG_LOG_ENABLED == 1)
    sys_fusion_log_process();
#endif
    OS_DELAY_MS(SYS_LOG_UPDATE_RATE_MS);
  }
}

void sys_cmd_thread_func(void *param)
{
  OS_SEM_CREATE(sys_cmd_req_sem);
  while (true)
  {
    sys_cmd_process();
  }
}

void sys_cmd_usb_thread_func(void *param)
{
  sys_cmd_usb_init();
  while (true)
  {
    sys_cmd_usb_process();
  }
}

void sys_manager_thread_func(void *param)
{
  sys_manager_init();

  while (true)
  {
    sys_manager_process();
  }
}

void sys_misc_thread_func(void *param)
{
  sys_buzzer_init();
  sys_buzzer_write_event(SYS_BUZZER_EVT_STARTUP);
  sys_led_init();

  while (true)
  {
    bsp_usb_process();
    sys_buzzer_process();
    sys_led_process();
    OS_DELAY_MS(SYS_MISC_UPDATE_RATE_MS);
  }
}

void sys_button_thread_func(void *param)
{
  sys_button_init();

  while (true)
  {
    sys_button_process();
    OS_DELAY_MS(SYS_BUTTON_UPDATE_RATE_MS);
  }
}

void sys_error_thread_func(void *param)
{
  sys_error_init();

  while (true)
  {
    sys_error_process();
    OS_DELAY_MS(SYS_ERROR_UPDATE_RATE_MS);
  }
}

void sys_ble_thread_func(void *param)
{
  // sys_network_adapter_ble_init() called in setup() — BLE controller OOM if init is inside task.
  // sys_network_adapter_ble_process() internally calls ble_adapter_poll() (blocks up to 20 ms).
  while (true)
  {
    sys_network_adapter_ble_process();
  }
}

void kill_all_threads(void)
{
#if (DEVICE_INPUT_ENABLED)
  OS_THREAD_DELETE(sys_input_thread);
#endif
#if (DEVICE_NETWORK_ENABLED && (DEVICE_FUSION_TUNING_MODE_ENABLED != 1))
  OS_THREAD_DELETE(sys_network_thread);
  OS_THREAD_DELETE(sys_network_lte_thread);
#endif
#if (DEVICE_UI_ENABLED)
  OS_THREAD_DELETE(sys_ui_thread);
#endif
  OS_THREAD_DELETE(sys_log_thread);
  OS_THREAD_DELETE(sys_misc_thread);
  OS_THREAD_DELETE(sys_cmd_thread);
  OS_THREAD_DELETE(sys_cmd_usb_thread);
  OS_THREAD_DELETE(sys_manager_thread);
  OS_THREAD_DELETE(sys_button_thread);
  OS_THREAD_DELETE(sys_ble_thread);
}

/* End of file -------------------------------------------------------- */
