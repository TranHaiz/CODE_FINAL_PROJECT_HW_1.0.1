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
#include "bsp_dust_sensor.h"

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
OS_THREAD_DECLARE(thread3, tskIDLE_PRIORITY + 1, 16384); /* LVGL requires larger stack */
OS_THREAD_DECLARE(thread4, tskIDLE_PRIORITY + 2, 4096); /* Dust sensor thread */

void gps_position_callback(bsp_gps_data_t *data)
{
  Serial.printf("Lat: %.6f, Lng: %.6f\n", data->latitude, data->longitude);
}

void dust_callback(bsp_dust_sensor_data_t *data) {
    Serial.printf("Dust: %d µg/m³\n", data->dust_density);
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

bsp_dust_sensor_t dust_sensor;
void thread4_func(void *param)
{
  if (bsp_dust_sensor_init(&dust_sensor) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to initialize dust sensor");
    return;
  }
  bsp_dust_sensor_register_callback(&dust_sensor, dust_callback);
  if (bsp_dust_sensor_begin(&dust_sensor) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to start dust sensor");
    return;
  }
  while (1)
  {
    bsp_dust_sensor_update(&dust_sensor);
    OS_DELAY_MS(1000);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(thread4, thread4_func);
}

void loop()
{
  // Main loop can be used for other tasks or left empty
  OS_DELAY_MS(1000);
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
