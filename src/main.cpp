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
#include "bsp_dust_sensor.h"
#include "bsp_error.h"
#include "bsp_gps.h"
#include "bsp_sim.h"
#include "common_type.h"
#include "device_pin_conf.h"
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
OS_THREAD_DECLARE(thread3, tskIDLE_PRIORITY + 1, 16384); /* LVGL requires larger stack */
OS_THREAD_DECLARE(thread4, tskIDLE_PRIORITY + 2, 4096);  /* Dust sensor thread */
OS_THREAD_DECLARE(thread5, tskIDLE_PRIORITY + 2, 4096);  /* Accelerometer thread */

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

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("START");
  delay(1000);  // Wait for Serial to initialize
  OS_THREAD_CREATE(thread5, thread5_func);
}

void loop()
{
  // Main loop can be used for other tasks or left empty
  OS_DELAY_MS(1000);
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
