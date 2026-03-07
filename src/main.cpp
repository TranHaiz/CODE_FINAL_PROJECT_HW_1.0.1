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
#include "bsp_error.h"
#include "bsp_gps.h"
#include "bsp_sim.h"
#include "common_type.h"
#include "os_lib.h"
#include "sys_ui.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
status_function_t g_ret = STATUS_ERROR;
char              g_buffer[256];
uint16_t          g_data_len = 0;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
OS_THREAD_DECLARE(thread1, tskIDLE_PRIORITY + 2, 4096);
OS_THREAD_DECLARE(thread2, tskIDLE_PRIORITY + 2, 4096);
OS_THREAD_DECLARE(thread3, tskIDLE_PRIORITY + 1, 4096);

void gps_position_callback(bsp_gps_data_t *data)
{
  Serial.printf("Lat: %.6f, Lng: %.6f\n", data->latitude, data->longitude);
}

void thread2_func(void *param)
{
  bsp_gps_init(gps_position_callback);
  while (1)
  {
    OS_DELAY_MS(1000);
  }
}

void thread3_func(void *param)
{
  sys_ui_begin(&g_ui);
  while (1)
  {
    sys_ui_run(&g_ui);
    OS_YIELD();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(thread3, thread3_func);
}

void loop()
{
  // Main loop can be used for other tasks or left empty
  OS_DELAY_MS(1000);
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
