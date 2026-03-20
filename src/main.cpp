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
#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "bsp_sim.h"
#include "common_type.h"
#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_input.h"
#include "sys_log.h"
#include "sys_network.h"
#include "sys_ui_simple.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INFO)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
status_function_t g_ret = STATUS_ERROR;
sys_input_data_t  g_input_data;

OS_THREAD_DECLARE(sys_input_thread, tskIDLE_PRIORITY + 2, 4096);
OS_THREAD_DECLARE(sys_network_thread, tskIDLE_PRIORITY + 3, 8192);
OS_THREAD_DECLARE(sys_ui_thread, tskIDLE_PRIORITY + 2, 16384);
OS_THREAD_DECLARE(sys_log_thread, tskIDLE_PRIORITY + 4, 4096);

/* Private function prototypes ---------------------------------------- */
void sys_input_thread_func(void *param);
void sys_network_thread_func(void *param);
void sys_ui_thread_func(void *param);
void sys_log_thread_func(void *param);

/* Function definitions ----------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  bsp_rtc_init();
  bsp_sdcard_init();
  delay(1000);
  Serial.println("START");
  delay(1000);
  OS_THREAD_CREATE(sys_input_thread, sys_input_thread_func);
  OS_THREAD_CREATE(sys_network_thread, sys_network_thread_func);
  OS_THREAD_CREATE(sys_ui_thread, sys_ui_thread_func);
  OS_THREAD_CREATE(sys_log_thread, sys_log_thread_func);
}

void loop()
{
  OS_DELAY_MS(OS_MAX_DELAY);
}

/* Private definitions ----------------------------------------------- */
void sys_input_thread_func(void *param)
{
  Serial.println("System input thread started");
  // Initialize system input
  sys_input_init();
  Serial.println("System input initialized");

  while (true)
  {
    // Process sensor data
    g_ret = sys_input_process();
    if (g_ret != STATUS_OK)
    {
      Serial.println("Error processing sensor data");
    }
    else
    {
      // Get current data
      if (sys_input_get_data(&g_input_data) == STATUS_OK)
      {
        is_data_network_ready = true;
        is_ui_data_ready      = true;
      }
    }

    OS_DELAY_MS(SYS_INPUT_UPDATE_RATE_MS);
  }
}

void sys_network_thread_func(void *param)
{
  sys_network_init();

  while (true)
  {
    sys_network_process();
    OS_DELAY_MS(100);
  }
}

void sys_ui_thread_func(void *param)
{
  sys_ui_simple_init();

  while (true)
  {
    sys_ui_simple_process();
    OS_DELAY_MS(10);
  }
}

void sys_log_thread_func(void *param)
{
  sys_log_init();

  while (true)
  {
    sys_log_process();
    OS_DELAY_MS(200);
  }
}

/* End of file -------------------------------------------------------- */
