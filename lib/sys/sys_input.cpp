/**
 * @file       sys_input.cpp
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.1.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      System Input Layer - Sensor data management implementation
 *
 * @details    Manages input from sensors and aggregates data from sys_fusion,
 *             dust sensor, temperature/humidity, and battery monitoring
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_input.h"

#include "bsp_acc.h"
#include "bsp_batt.h"
#include "bsp_dust_sensor.h"
#include "bsp_io.h"
#include "bsp_temp_hum.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_fusion.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_input, LOG_LEVEL_DBG)

#define SYS_INPUT_DUST_EMA_ALPHA       (0.2f)

#define SYS_INPUT_BATT_INITIAL_SAMPLES (200)
#define SYS_INPUT_BATT_EMA_ALPHA       (0.1f)
#define SYS_INPUT_BATT_READ_VOLT_TIMES (20)
#define SYS_INPUT_BATT_DEBOUNCE        (10)
#define SYS_INPUT_BATT_MAX_ERROR       (5)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  bool dust_ready;
  bool temp_hum_ready;
  bool initialized;

  uint32_t last_env_update_ms;

  int32_t  batt_remaining_mah;
  uint32_t batt_last_update_ms;

  sys_input_data_t data;
} sys_input_context_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static sys_input_context_t input_ctx = { 0 };

/* Private function prototypes ---------------------------------------- */
static void sys_input_read_dust_sensor(void);
static void sys_input_initial_battery_level(void);
static void sys_input_read_battery_level(float *battery_level);

/* Function definitions ----------------------------------------------- */

void sys_input_init(void)
{
  if (input_ctx.initialized)
    return;

  LOG_DBG("Initializing system input...");

  input_ctx.data.velocity_ms          = 0.0f;
  input_ctx.data.velocity_kmh         = 0.0f;
  input_ctx.data.distance_m           = 0.0f;
  input_ctx.data.dust_value           = 0.0f;
  input_ctx.data.temp_hum.temperature = 0.0f;
  input_ctx.data.temp_hum.humidity    = 0.0f;
  input_ctx.data.timestamp_ms         = 0;
  input_ctx.data.heading_deg          = 0.0f;
  input_ctx.data.direction_str        = "N";
  input_ctx.data.battery_level        = 100.0f;

  input_ctx.last_env_update_ms = 0;
  input_ctx.dust_ready         = false;
  input_ctx.temp_hum_ready     = false;
  input_ctx.initialized        = true;

  LOG_DBG("Init Battery");
  if (bsp_batt_init() != STATUS_OK)
  {
    LOG_ERR("Failed to initialize battery monitoring");
  }
  sys_input_initial_battery_level();

  LOG_DBG("Init Dust");
  if (bsp_dust_sensor_init() == STATUS_OK)
  {
    input_ctx.dust_ready = true;
    LOG_DBG("Dust OK");
  }

  LOG_DBG("Init Temp/Hum");
  if (bsp_temp_hum_init() == STATUS_OK)
  {
    input_ctx.temp_hum_ready = true;
    LOG_DBG("Temp/Hum OK");
  }

  // Sensor fusion: ACC, GPS, Compass
  sys_fusion_init();
}

status_function_t sys_input_process(void)
{
  if (!input_ctx.initialized)
    return STATUS_ERROR;

  size_t current_time_ms = millis();

  // 1. Sensor fusion: velocity, distance, heading, GPS position
  sys_fusion_data_t fusion_data = { 0 };
  fusion_data.direction_str     = "N";
  sys_fusion_process(&fusion_data);

  input_ctx.data.velocity_ms   = fusion_data.velocity_ms;
  input_ctx.data.velocity_kmh  = fusion_data.velocity_kmh;
  input_ctx.data.distance_m    = fusion_data.distance_m;
  input_ctx.data.heading_deg   = fusion_data.heading_deg;
  input_ctx.data.direction_str = fusion_data.direction_str;
  input_ctx.data.gps_position  = fusion_data.gps_position;

  // 2. Environmental sensors
  if ((current_time_ms - input_ctx.last_env_update_ms) >= SYS_INPUT_ENV_UPDATE_RATE_MS)
  {
    input_ctx.last_env_update_ms = current_time_ms;
    sys_input_read_dust_sensor();
    if (input_ctx.temp_hum_ready)
    {
      (void) bsp_temp_hum_read(&input_ctx.data.temp_hum);
    }
  }

  // 3. Battery level
  if ((current_time_ms - input_ctx.batt_last_update_ms) >= SYS_INPUT_BATT_UPDATE_RATE_MS)
  {
    input_ctx.batt_last_update_ms = current_time_ms;
    sys_input_read_battery_level(&input_ctx.data.battery_level);
  }

  // 4. Finalize
  input_ctx.data.timestamp_ms = current_time_ms;

  return STATUS_OK;
}

status_function_t sys_input_get_data(sys_input_data_t *data)
{
  if (data == NULL || !input_ctx.initialized)
    return STATUS_ERROR;
  *data = input_ctx.data;
  return STATUS_OK;
}

status_function_t sys_input_enter_sleep_mode(void)
{
  status_function_t status = STATUS_OK;

  status = bsp_acc_enable_interrupt(BSP_ACC_INT_PIN_1);
  return status;
}

/* Private definitions ----------------------------------------------- */

static void sys_input_read_dust_sensor(void)
{
  if (!input_ctx.dust_ready)
  {
    input_ctx.data.dust_value = 0.0f;
    return;
  }

  bsp_dust_sensor_data_t dust_data;
  if (bsp_dust_sensor_read(&dust_data) != STATUS_OK)
    return;

  float new_value = (float) dust_data.running_average;
  input_ctx.data.dust_value =
    SYS_INPUT_DUST_EMA_ALPHA * new_value + (1.0f - SYS_INPUT_DUST_EMA_ALPHA) * input_ctx.data.dust_value;
}

static void sys_input_read_battery_level(float *battery_level)
{
  static uint8_t err_cnt = 0;
  uint32_t       now     = OS_GET_TICK();
  size_t         dt_ms   = now - input_ctx.batt_last_update_ms;

  float raw_ma   = bsp_batt_read_current_ma();
  *battery_level = (SYS_INPUT_BATT_EMA_ALPHA * raw_ma) + ((1.0f - SYS_INPUT_BATT_EMA_ALPHA) * (*battery_level));

  float delta_mah = (*battery_level) * (dt_ms / 3600000.0f);
  input_ctx.batt_remaining_mah -= delta_mah;
  input_ctx.batt_remaining_mah = input_ctx.batt_remaining_mah < 0.0f ? 0.0f
                                 : input_ctx.batt_remaining_mah > BSP_BATTERY_CAPACITY_MAH
                                   ? BSP_BATTERY_CAPACITY_MAH
                                   : input_ctx.batt_remaining_mah;

  float   voltage_mv      = 0.0f;
  float   sum_voltage_mv  = 0.0f;
  uint8_t check_err_count = 0;
  for (int i = 0; i < SYS_INPUT_BATT_READ_VOLT_TIMES; i++)
  {
    voltage_mv = bsp_batt_read_voltage_mv();

    if (voltage_mv <= 0.0f)
    {
      LOG_ERR("Battery voltage read 0 volt");
      check_err_count++;
    }
    else
    {
      sum_voltage_mv += voltage_mv;
    }

    if (check_err_count > SYS_INPUT_BATT_DEBOUNCE)
    {
      // Not update battery level if too many read errors
      err_cnt++;
      if (err_cnt > SYS_INPUT_BATT_MAX_ERROR)
      {
        err_cnt = 0;
        LOG_ERR("Too many consecutive battery read errors, resetting remaining capacity to 0");
        // Handle error battery level if too many consecutive errors
      }
      return;
    }
    OS_YIELD();
  }
  voltage_mv = sum_voltage_mv / ((float) SYS_INPUT_BATT_READ_VOLT_TIMES);

  if (voltage_mv >= BSP_BATT_VOLTAGE_FULL_MV)
  {
    input_ctx.batt_remaining_mah = BSP_BATTERY_CAPACITY_MAH;
    *battery_level               = 100.0f;
    return;
  }
  if (voltage_mv <= BSP_BATT_VOLTAGE_EMPTY_MV)
  {
    input_ctx.batt_remaining_mah = 0.0f;
    *battery_level               = 0.0f;
    return;
  }

  float soc_coulomb = (input_ctx.batt_remaining_mah / BSP_BATTERY_CAPACITY_MAH) * 100.0f;
  float soc_voltage =
    ((float) (voltage_mv - BSP_BATT_VOLTAGE_EMPTY_MV) / (float) (BSP_BATT_VOLTAGE_FULL_MV - BSP_BATT_VOLTAGE_EMPTY_MV))
    * 100.0f;

  float soc = (0.9f * soc_coulomb) + (0.1f * soc_voltage);
  soc       = soc < 0.0f ? 0.0f : soc > 100.0f ? 100.0f : soc;

  *battery_level = soc;
}

static void sys_input_initial_battery_level(void)
{
  float sum = 0.0f;

  for (int i = 0; i < SYS_INPUT_BATT_INITIAL_SAMPLES; i++)
  {
    int32_t m_volt = bsp_batt_read_voltage_mv();

    if (m_volt >= BSP_BATT_VOLTAGE_FULL_MV)
      sum += 100.0f;
    else if (m_volt <= BSP_BATT_VOLTAGE_EMPTY_MV)
      sum += 0.0f;
    else
      sum +=
        ((float) (m_volt - BSP_BATT_VOLTAGE_EMPTY_MV) / (float) (BSP_BATT_VOLTAGE_FULL_MV - BSP_BATT_VOLTAGE_EMPTY_MV))
        * 100.0f;

    OS_YIELD();
  }

  float initial_soc = sum / SYS_INPUT_BATT_INITIAL_SAMPLES;

  input_ctx.batt_remaining_mah = (initial_soc / 100.0f) * BSP_BATTERY_CAPACITY_MAH;
  input_ctx.data.battery_level = initial_soc;

  LOG_INF("Initial battery level: %.2f%%", initial_soc);
}

/* End of file -------------------------------------------------------- */
