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
#include "bsp_buzzer.h"
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
#include "sys_cmd.h"
#include "sys_cmd_usb.h"
#include "sys_input.h"
#include "sys_log.h"
#include "sys_manager.h"
#include "sys_network.h"
#include "sys_ui.h"
#include "sys_ui_simple.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INFO)

#define SYS_INPUT_UPDATE_RATE_MS (20)
#define SYS_UI_UPDATE_RATE_MS    (10)
#define SYS_LOG_UPDATE_RATE_MS   (500)
#define SYS_MISC_UPDATE_RATE_MS  (100)
#define LED_RGB_TASK_MS          (100)

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
OS_THREAD_DECLARE(sys_network_thread, tskIDLE_PRIORITY + 2, 8192);
OS_THREAD_DECLARE(network_data_thread, tskIDLE_PRIORITY + 3, 6144);
#endif

#if (DEVICE_UI_ENABLED)
OS_THREAD_DECLARE(sys_ui_thread, tskIDLE_PRIORITY + 4, 16384);
#endif

OS_THREAD_DECLARE(sys_log_thread, tskIDLE_PRIORITY + 1, 4096);
OS_THREAD_DECLARE(sys_misc_thread, tskIDLE_PRIORITY + 4, 4096);
OS_THREAD_DECLARE(sys_cmd_thread, tskIDLE_PRIORITY + 5, 4096);
OS_THREAD_DECLARE(sys_cmd_usb_thread, tskIDLE_PRIORITY + 5, 4096);
OS_THREAD_DECLARE(sys_manager_thread, tskIDLE_PRIORITY + 5, 4096);

/* Private function prototypes ---------------------------------------- */
void sys_input_thread_func(void *param);
void sys_ui_thread_func(void *param);
void sys_log_thread_func(void *param);
void sys_cmd_thread_func(void *param);
void sys_cmd_usb_thread_func(void *param);
void sys_manager_thread_func(void *param);
void sys_misc_thread_func(void *param);
void callback_button_press(void);

/* Function definitions ----------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  bsp_rtc_init();
  bsp_sdcard_init();
  delay(1000);
  device_info_init();
  sys_network_init();
  delay(1000);
  OS_THREAD_CREATE(sys_cmd_thread, sys_cmd_thread_func);
  OS_THREAD_CREATE(sys_cmd_usb_thread, sys_cmd_usb_thread_func);

#if (DEVICE_INPUT_ENABLED)
  OS_THREAD_CREATE(sys_input_thread, sys_input_thread_func);
#endif

#if (DEVICE_NETWORK_ENABLED)
  OS_THREAD_CREATE(sys_network_thread, sys_network_process);
  OS_THREAD_CREATE(network_data_thread, sys_network_data_task);
#endif

#if (DEVICE_UI_ENABLED)
  OS_THREAD_CREATE(sys_ui_thread, sys_ui_thread_func);
#endif

  OS_THREAD_CREATE(sys_log_thread, sys_log_thread_func);
  OS_THREAD_CREATE(sys_misc_thread, sys_misc_thread_func);
  OS_THREAD_CREATE(sys_manager_thread, sys_manager_thread_func);
}

void loop()
{
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

  while (true)
  {
    sys_log_process();
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
  bsp_buzzer_init();
  bsp_io_int_init(IO_BUTTON_PIN, BSP_IO_EVENT_FALLING, callback_button_press);
  bsp_buzzer_beep_cycle(3, 200, 500);
  bsp_led_init(LED_RGB_TASK_MS);
  bsp_led_off();

  while (true)
  {
    bsp_usb_process();
    bsp_buzzer_process();
    bsp_led_task();
    OS_DELAY_MS(SYS_MISC_UPDATE_RATE_MS);
  }
}

void callback_button_press(void)
{
  LOG_DBG("Button pressed!");
}

/* End of file -------------------------------------------------------- */
