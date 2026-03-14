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

#if (CONFIG_MQTT_SERVER == true)
mqtt_message_t g_mqtt_msg = { .topic = "haq-trk-001/topic", .payload = "Hello from esp32" };

#endif  // CONFIG_MQTT_SERVER

OS_THREAD_DECLARE(sys_input_thread, tskIDLE_PRIORITY + 2, 4096);
OS_THREAD_DECLARE(sys_network_thread, tskIDLE_PRIORITY + 3, 8192);

/* Private function prototypes ---------------------------------------- */
void sys_input_thread_func(void *param);
void sys_network_thread_func(void *param);

/* Function definitions ----------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(sys_input_thread, sys_input_thread_func);
  // OS_THREAD_CREATE(sys_network_thread, sys_network_thread_func);
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

void sys_network_thread_func(void *param)
{
  LOG_INF("System network thread started");
  bsp_sim_init();
  if (bsp_sim_mqtt_deinit() != STATUS_OK)
  {
    // Do nothing
  }
  if (bsp_sim_mqtt_init() != STATUS_OK)
  {
    LOG_ERR("Failed to initialize MQTT");
    while (1)
    {
      OS_DELAY_MS(1000);
    }
  }
  else
  {
    bsp_sim_mqtt_sub("haq-trk-001/command", [](const char *topic, const uint8_t *data, size_t len)
                     { LOG_INF("Received MQTT message on topic %s: %.*s", topic, (int) len, data); });
  }
  LOG_INF("MQTT initialized");
  OS_DELAY_MS(2000);  // Wait for MQTT to stabilize
  while (1)
  {
    bsp_sim_mqtt_pub(&g_mqtt_msg);
    OS_DELAY_MS(1000);
  }
}

/* End of file -------------------------------------------------------- */
