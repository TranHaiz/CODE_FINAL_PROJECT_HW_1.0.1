/**
 * @file       example_dust_sensor.cpp
 * @brief      Example usage of BSP Dust Sensor API
 * 
 * This example demonstrates how to use the bsp_dust_sensor API with GP2Y1010AU0F
 */

#include "bsp_dust_sensor.h"
#include "os_lib.h"
#include <Arduino.h>

/* Global dust sensor context */
bsp_dust_sensor_t g_dust_sensor;

/* Callback function - called automatically when new data is available */
void dust_sensor_callback(bsp_dust_sensor_data_t *data)
{
  if (data == nullptr || !data->is_valid)
  {
    return;
  }

  // Get AQI level
  bsp_dust_aqi_level_t aqi_level;
  bsp_dust_sensor_get_aqi_level(data->dust_density, &aqi_level);

  const char *aqi_str[] = {"EXCELLENT", "GOOD", "MODERATE", "POOR", "UNHEALTHY", "HAZARDOUS"};

  Serial.printf("[DUST] Density: %d ug/m3 | Avg: %d ug/m3 | AQI: %s | Baseline: %.2fV\n", 
                data->dust_density, 
                data->running_average, 
                aqi_str[aqi_level],
                data->baseline_voltage);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Dust Sensor Example ===");

  // Method 1: Initialize with default configuration
  if (bsp_dust_sensor_init(&g_dust_sensor) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to initialize dust sensor");
    return;
  }

  // Optional: Configure custom settings
  // bsp_dust_sensor_set_baseline(&g_dust_sensor, 0.45f);      // Set baseline voltage
  // bsp_dust_sensor_set_calibration(&g_dust_sensor, 1.05f);   // Set calibration factor
  // bsp_dust_sensor_set_sensitivity(&g_dust_sensor, 0.5f);    // Set sensitivity

  // Register callback for automatic updates
  bsp_dust_sensor_register_callback(&g_dust_sensor, dust_sensor_callback);

  // Start sensor
  if (bsp_dust_sensor_begin(&g_dust_sensor) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to start dust sensor");
    return;
  }

  Serial.println("[OK] Dust sensor started successfully");
}

void loop()
{
  // Method 1: Automatic update with callback (recommended)
  bsp_dust_sensor_update(&g_dust_sensor);  // Call this periodically
  
  // Method 2: Manual reading without callback
  // bsp_dust_sensor_read(&g_dust_sensor);
  // uint16_t density = 0;
  // bsp_dust_sensor_get_density(&g_dust_sensor, &density);
  // Serial.printf("Dust density: %d ug/m3\n", density);

  OS_DELAY_MS(100);  // Short delay for main loop
}

/* Alternative: Custom configuration example */
void example_custom_config()
{
  bsp_dust_sensor_config_t custom_config = {
    .led_pin           = 15,      // Custom LED pin
    .analog_pin        = 4,       // Custom analog pin
    .sample_count      = 30,      // More samples per reading
    .avg_buffer_size   = 120,     // Larger averaging buffer (2 minutes at 1Hz)
    .baseline_voltage  = 0.42f,   // Measured baseline for your sensor
    .calibration_factor = 1.08f,  // Calibrated against reference device
    .update_rate_ms    = 500,     // Update every 500ms (2Hz)
  };

  bsp_dust_sensor_t sensor;
  bsp_dust_sensor_init_with_config(&sensor, &custom_config);
  bsp_dust_sensor_begin(&sensor);
}
