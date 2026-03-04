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
#include "bsp_sim.h"
#include "common_type.h"
#include "os_lib.h"
#include "bsp_error.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
status_function_t g_ret         = STATUS_ERROR;
char              g_buffer[256];
uint16_t          g_data_len    = 0;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
OS_THREAD_DECLARE(thread1, tskIDLE_PRIORITY + 1, 4096);

void thread1_func(void *param)
{
  g_ret = bsp_sim_init();

  while (1)
  {
    if (g_ret != STATUS_OK)
    {
      bsp_error_handler(BSP_ERROR_SIM_INIT);
    }
    firebase_data_t data_firebase_test;
    data_firebase_test.batt_level         = rand() % 100 + 1;
    data_firebase_test.position.latitude  = (float) (rand() % 18000) / 100.0f - 90.0f;   // -90.0 to +90.0
    data_firebase_test.position.longitude = (float) (rand() % 36000) / 100.0f - 180.0f;  // -180.0 to +180.0
    data_firebase_test.speed              = (float) (rand() % 2000) / 10.0f;             // 0.0 to 200.0

    g_ret = bsp_sim_send_data_firebase(&data_firebase_test);
    g_ret = bsp_sim_get_raw_data_firebase((uint8_t *) g_buffer, &g_data_len);
    Serial.print("[INFO]:");
    Serial.print(g_buffer);
    Serial.println(", data_len: " + String(g_data_len));

    OS_DELAY_MS(1000);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(thread1, thread1_func);
}

void loop()
{
  // Main loop can be used for other tasks or left empty
  OS_DELAY_MS(1000);
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
