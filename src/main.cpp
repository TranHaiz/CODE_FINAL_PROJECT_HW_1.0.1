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
#include "bsp_acc.h"
#include "bsp_compass.h"
#include "bsp_dust_sensor.h"
#include "bsp_error.h"
#include "bsp_gps.h"
#include "bsp_sdcard.h"
#include "bsp_sim.h"
#include "common_type.h"
#include "device_pin_conf.h"
#include "os_lib.h"
#include "sys_ui.h"
#include "device_info.h"

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
OS_THREAD_DECLARE(thread4, tskIDLE_PRIORITY + 2, 4096);  /* Dust sensor thread */
OS_THREAD_DECLARE(thread5, tskIDLE_PRIORITY + 2, 4096);  /* Accelerometer thread */
OS_THREAD_DECLARE(thread6, tskIDLE_PRIORITY + 2, 4096);  /* Compass thread */
OS_THREAD_DECLARE(thread7, tskIDLE_PRIORITY + 2, 8192);  /* SD Card thread (larger stack) */

void gps_position_callback(bsp_gps_data_t *data)
{
  Serial.printf("Lat: %.6f, Lng: %.6f\n", data->latitude, data->longitude);
}

void dust_callback(bsp_dust_sensor_data_t *data)
{
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
void              thread4_func(void *param)
{
  if (bsp_dust_sensor_init(&dust_sensor) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to initialize dust sensor");
    vTaskDelete(NULL);
    return;
  }
  bsp_dust_sensor_register_callback(&dust_sensor, dust_callback);
  if (bsp_dust_sensor_begin(&dust_sensor) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to start dust sensor");
    vTaskDelete(NULL);
    return;
  }
  while (1)
  {
    bsp_dust_sensor_update(&dust_sensor);
    OS_DELAY_MS(1000);
  }
}

void acc_callback(bsp_acc_data_t *data)
{
  Serial.printf("Velocity: %.1f km/h\n", data->velocity_kmh);
}
bsp_acc_t accelerometer;
void      thread5_func(void *param)
{
  // Scan I2C bus first to debug hardware connection
  int8_t found_addr = bsp_acc_scan_i2c(ACC_I2C_SDA_PIN, ACC_I2C_SCL_PIN);
  if (found_addr < 0)
  {
    Serial.println("[ERROR] No accelerometer found on I2C bus");
    vTaskDelete(NULL);
    return;
  }

  if (bsp_acc_init(&accelerometer) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to initialize accelerometer");
    vTaskDelete(NULL);
    return;
  }
  if (bsp_acc_begin(&accelerometer) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to start accelerometer");
    vTaskDelete(NULL);
    return;
  }
  if (bsp_acc_calibrate(&accelerometer) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to calibrate accelerometer");
    vTaskDelete(NULL);
    return;
  }
  bsp_acc_register_callback(&accelerometer, acc_callback);

  while (1)
  {
    bsp_acc_update(&accelerometer);
    OS_DELAY_MS(100);
  }
}

void compass_callback(bsp_compass_data_t *data)
{
  Serial.printf(">>> %.0f° %s\n", data->heading_deg, data->direction_str);
}

bsp_compass_t compass;
void          thread6_func(void *param)
{
  // Scan I2C bus first to debug hardware connection
  int8_t found_addr = bsp_compass_scan_i2c(COMPASS_I2C_SDA_PIN, COMPASS_I2C_SCL_PIN);
  if (found_addr < 0)
  {
    Serial.println("[ERROR] No compass found on I2C bus");
    vTaskDelete(NULL);
    return;
  }

  if (bsp_compass_init(&compass) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to initialize compass");
    vTaskDelete(NULL);
    return;
  }

  if (bsp_compass_begin(&compass) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to start compass");
    vTaskDelete(NULL);
    return;
  }

  bsp_compass_register_callback(&compass, compass_callback);

  while (1)
  {
    bsp_compass_update(&compass);
    OS_DELAY_MS(500);
  }
}

bsp_sdcard_t sdcard;
void         thread7_func(void *param)
{
  Serial.println("[SDCARD] Starting SD Card CSV/LOG test...");

  // Initialize SD card with custom config
  bsp_sdcard_config_t sdcard_config = {
    .cs_pin        = BSP_SDCARD_DEFAULT_CS_PIN,
    .sck_pin       = BSP_SDCARD_DEFAULT_SCK_PIN,
    .miso_pin      = BSP_SDCARD_DEFAULT_MISO_PIN,
    .mosi_pin      = BSP_SDCARD_DEFAULT_MOSI_PIN,
    .spi_freq      = BSP_SDCARD_DEFAULT_SPI_FREQ,
    .mount_retries = 3,
  };

  if (bsp_sdcard_init_with_config(&sdcard, &sdcard_config) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to initialize SD card");
    vTaskDelete(NULL);
    return;
  }

  // Mount SD card
  if (bsp_sdcard_mount(&sdcard) != STATUS_OK)
  {
    Serial.println("[ERROR] Failed to mount SD card");
    vTaskDelete(NULL);
    return;
  }

  // Get and print card info
  bsp_sdcard_info_t info;
  if (bsp_sdcard_get_info(&sdcard, &info) == STATUS_OK)
  {
    Serial.printf("[SDCARD] Type: %s\n", bsp_sdcard_type_to_str(info.card_type));
    Serial.printf("[SDCARD] Size: %llu MB\n", info.card_size_bytes / (1024 * 1024));
    Serial.printf("[SDCARD] Free: %llu MB\n", info.free_bytes / (1024 * 1024));
  }

  // ═══════════════════════════════════════════════════════════════════════
  // TEST CSV FUNCTIONS
  // ═══════════════════════════════════════════════════════════════════════
  Serial.println("\n=== CSV FILE TEST ===");

  // Create CSV file with headers
  const char *csv_headers[] = { "timestamp_ms", "sensor_x", "sensor_y", "sensor_z", "temperature" };
  if (bsp_sdcard_csv_create(&sdcard, "/sensor_data.csv", csv_headers, 5) == STATUS_OK)
  {
    Serial.println("[CSV] Created sensor_data.csv with headers");
  }

  // Append some rows with float data
  for (int i = 0; i < 5; i++)
  {
    float values[] = {
      (float) (i * 0.5f + 1.0f),   // sensor_x
      (float) (i * -0.3f + 2.5f),  // sensor_y
      (float) (9.8f + i * 0.01f),  // sensor_z
      (float) (25.0f + i * 0.1f)   // temperature
    };
    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "%lu", millis());
    bsp_sdcard_csv_append_timestamped(&sdcard, "/sensor_data.csv", timestamp, values, 4, 3);
    Serial.printf("[CSV] Appended row %d\n", i + 1);
    OS_DELAY_MS(100);
  }

  // Append row with string values
  const char *str_values[] = { "1000", "1.5", "2.3", "9.81", "26.5" };
  bsp_sdcard_csv_append_row(&sdcard, "/sensor_data.csv", str_values, 5);
  Serial.println("[CSV] Appended string row");

  // Append row with integers
  int32_t int_values[] = { 2000, 100, 200, 981, 27 };
  bsp_sdcard_csv_append_ints(&sdcard, "/sensor_data.csv", int_values, 5);
  Serial.println("[CSV] Appended integer row");

  // Read back CSV file
  Serial.println("\n[CSV] Contents of sensor_data.csv:");
  char   csv_buf[512] = { 0 };
  size_t csv_len      = 0;
  if (bsp_sdcard_read_file(&sdcard, "/sensor_data.csv", (uint8_t *) csv_buf, sizeof(csv_buf) - 1, &csv_len)
      == STATUS_OK)
  {
    Serial.println(csv_buf);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // TEST LOG FUNCTIONS
  // ═══════════════════════════════════════════════════════════════════════
  Serial.println("\n=== LOG FILE TEST ===");

  // Create LOG file
  status_function_t log_status = bsp_sdcard_log_create(&sdcard, "/system.log", "System Event Log - Test Session");
  if (log_status == STATUS_OK)
  {
    Serial.println("[LOG] Created system.log");

    // Verify file exists
    if (bsp_sdcard_file_exists(&sdcard, "/system.log"))
    {
      Serial.println("[LOG] File verified to exist");
    }
    else
    {
      Serial.println("[LOG] WARNING: File not found after creation!");
    }

    // Write log messages with different levels
    bsp_sdcard_log_write(&sdcard, "/system.log", BSP_SDCARD_LOG_INFO, "System initialized");
    bsp_sdcard_log_write(&sdcard, "/system.log", BSP_SDCARD_LOG_DEBUG, "Debug mode enabled");
    bsp_sdcard_log_write(&sdcard, "/system.log", BSP_SDCARD_LOG_WARNING, "Low memory warning");
    bsp_sdcard_log_write(&sdcard, "/system.log", BSP_SDCARD_LOG_ERROR, "Sensor timeout error");
    Serial.println("[LOG] Written 4 log entries");

    // Write timestamped log entries
    char ts1[32], ts2[32];
    snprintf(ts1, sizeof(ts1), "%lu ms", millis());
    bsp_sdcard_log_write_timestamped(&sdcard, "/system.log", ts1, BSP_SDCARD_LOG_INFO, "Timestamped entry 1");
    OS_DELAY_MS(500);
    snprintf(ts2, sizeof(ts2), "%lu ms", millis());
    bsp_sdcard_log_write_timestamped(&sdcard, "/system.log", ts2, BSP_SDCARD_LOG_INFO, "Timestamped entry 2");
    Serial.println("[LOG] Written 2 timestamped entries");

    // Write formatted log using printf
    float sensor_value = 23.456f;
    int   error_code   = 42;
    bsp_sdcard_log_printf(&sdcard, "/system.log", BSP_SDCARD_LOG_INFO, "Sensor reading: %.2f", sensor_value);
    bsp_sdcard_log_printf(&sdcard, "/system.log", BSP_SDCARD_LOG_ERROR, "Error code: %d", error_code);
    bsp_sdcard_log_printf(&sdcard, "/system.log", BSP_SDCARD_LOG_CRITICAL, "Critical failure at addr 0x%04X", 0xDEAD);
    Serial.println("[LOG] Written 3 printf formatted entries");

    // Read back LOG file
    Serial.println("\n[LOG] Contents of system.log:");
    char   log_buf[1024] = { 0 };
    size_t log_len       = 0;
    if (bsp_sdcard_read_file(&sdcard, "/system.log", (uint8_t *) log_buf, sizeof(log_buf) - 1, &log_len) == STATUS_OK)
    {
      Serial.println(log_buf);
    }
  }
  else
  {
    Serial.println("[LOG] ERROR: Failed to create system.log");
  }

  // ═══════════════════════════════════════════════════════════════════════
  // LIST FILES
  // ═══════════════════════════════════════════════════════════════════════
  Serial.println("\n=== DIRECTORY LISTING ===");
  bsp_sdcard_list_dir(&sdcard, "/");

  Serial.println("\n[SDCARD] CSV/LOG Test completed!");

  // Keep task alive
  while (1)
  {
    OS_DELAY_MS(10000);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(thread3, thread3_func);
  OS_THREAD_CREATE(thread7, thread7_func);
}

void loop()
{
  // Main loop can be used for other tasks or left empty
  OS_DELAY_MS(1000);
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
