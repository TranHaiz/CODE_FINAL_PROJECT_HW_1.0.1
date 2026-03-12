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

#include "common_type.h"
#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_input.h"
#include "sys_ui.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INFO)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
status_function_t g_ret = STATUS_ERROR;
char              g_buffer[256];
uint16_t          g_data_len = 0;
sys_input_data_t  g_input_data;

OS_THREAD_DECLARE(sys_input_thread, tskIDLE_PRIORITY + 2, 4096);

/* Private function prototypes ---------------------------------------- */
void sys_input_thread_func(void *param);

/* Function definitions ----------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(sys_input_thread, sys_input_thread_func);
}

void loop()
{
  // Main loop can be used for other tasks or left empty
  OS_DELAY_MS(1000);
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
        LOG_INF("Velocity: %.2f", g_input_data.velocity_ms);
        LOG_INF("Distance: %.2f", g_input_data.distance_m);
        LOG_INF("Dust: %.2f", g_input_data.dust_concentration);
        LOG_INF("Heading: %.2f %s", g_input_data.heading_deg, g_input_data.direction_str);
        LOG_INF("Temp: %.2f C, Humidity: %.2f %%", g_input_data.temp_hum.temperature, g_input_data.temp_hum.humidity);
        LOG_INF("GPS: Lat (%.6f, %.6f)", g_input_data.gps_position.latitude, g_input_data.gps_position.longitude);
      }
    }

    OS_DELAY_MS(SYS_INPUT_UPDATE_RATE_MS);
  }
}

/* End of file -------------------------------------------------------- */
