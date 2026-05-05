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
#include "sys_manager.h"
#include "sys_ui.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_input, LOG_LEVEL_SYS_INPUT)

#define SYS_INPUT_DUST_EMA_ALPHA       (0.2f)

#define SYS_INPUT_BATT_INITIAL_SAMPLES (200)
#define SYS_INPUT_BATT_EMA_ALPHA       (0.1f)
#define SYS_INPUT_BATT_READ_VOLT_TIMES (20)
#define SYS_INPUT_BATT_DEBOUNCE        (10)
#define SYS_INPUT_BATT_MAX_ERROR       (5)

#define SYS_INPUT_BATT_ENABLE          (true)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  bool dust_ready;
  bool temp_hum_ready;
  bool initialized;

  uint32_t last_dust_update_ms;
  uint32_t last_temp_hum_update_ms;

  int32_t  batt_remaining_mah;
  uint32_t batt_last_update_ms;
  float    prev_distance_m;

  sys_input_data_t data;
} sys_input_context_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sys_input_context_t input_ctx = { 0 };
OS_SEM_DEFINE_GLOBAL(sys_input_wakeup_sem);

/* Private function prototypes ---------------------------------------- */
static status_function_t sys_input_process_active(void);
static void              sys_input_process_idle(void);
static void              sys_input_process_locked(void);
static void              sys_input_read_dust_sensor(void);
static void              sys_input_initial_battery_level(void);
static void              sys_input_read_battery_level(float *battery_level);
static void              sys_input_wakeup_acc_handler(void);

/* Function definitions ----------------------------------------------- */
void sys_input_init(void)
{
  if (input_ctx.initialized)
    return;

  LOG_DBG("Initializing system input...");

  OS_SEM_CREATE(sys_input_wakeup_sem);
  memset(&input_ctx, 0, sizeof(input_ctx));
  input_ctx.data.direction_str = "N";
  input_ctx.data.battery_level = 100.0f;
  input_ctx.initialized        = true;

#if SYS_INPUT_BATT_ENABLE
  LOG_DBG("Init Battery");
  if (bsp_batt_init() != STATUS_OK)
  {
    LOG_ERR("Failed to initialize battery monitoring");
  }
  sys_input_initial_battery_level();
#else
  // Do nothing
#endif

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

  // Init IO interrupt for wakeup
  bsp_io_int_init(ACC_INT_PIN, BSP_IO_EVENT_RISING, sys_input_wakeup_acc_handler);
  bsp_acc_config_interrupt(BSP_ACC_INT_PIN_1, BSP_ACC_INT_MOTION_DETECT);
}

status_function_t sys_input_process(void)
{
  switch (g_device_info.nvs_info.curr_state)
  {
  case DEVICE_STATE_ACTIVE:
  {
    return sys_input_process_active();
  }
  case DEVICE_STATE_IDLE:
  {
    sys_input_process_idle();
    break;
  }
  case DEVICE_STATE_LOCKED:
  {
    sys_input_process_locked();
    return STATUS_BUSY;
  }
  default: break;
  }
}

status_function_t sys_input_get_data(sys_input_data_t *data)
{
  if (data == NULL || !input_ctx.initialized)
    return STATUS_ERROR;
  *data = input_ctx.data;
  return STATUS_OK;
}

status_function_t sys_input_get_fusion_data(sys_fusion_data_t *data)
{
  if (data == NULL || !input_ctx.initialized)
    return STATUS_ERROR;

  data->velocity_ms   = input_ctx.data.velocity_ms;
  data->velocity_kmh  = input_ctx.data.velocity_kmh;
  data->distance_m    = input_ctx.data.distance_m;
  data->heading_deg   = input_ctx.data.heading_deg;
  data->direction_str = input_ctx.data.direction_str;
  data->gps_position  = input_ctx.data.gps_position;
#if (DEVICE_FUSION_DEBUG_MODE == 1)
  data->debug = input_ctx.data.debug;
#endif

#if (DEVICE_NETWORK_TOTAL_KM_ENABLED)
  if (input_ctx.prev_distance_m < input_ctx.data.distance_m)
  {
    input_ctx.prev_distance_m = input_ctx.data.distance_m;
    g_device_info.nvs_info.total_km += (input_ctx.data.distance_m / 1000.0f);
  }
#endif

  return STATUS_OK;
}

status_function_t sys_input_get_env_data(sys_input_data_t *data)
{
  if (data == NULL || !input_ctx.initialized)
    return STATUS_ERROR;

  data->dust_value           = input_ctx.data.dust_value;
  data->temp_hum.temperature = input_ctx.data.temp_hum.temperature;
  data->temp_hum.humidity    = input_ctx.data.temp_hum.humidity;

  return STATUS_OK;
}

status_function_t sys_input_enter_sleep_mode(void)
{
  status_function_t status = STATUS_OK;

  status = bsp_acc_enable_interrupt(BSP_ACC_INT_PIN_1);
  return status;
}

void sys_input_wakeup(void)
{
  OS_SEM_GIVE(sys_input_wakeup_sem);
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

static status_function_t sys_input_process_active(void)
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
#if (DEVICE_FUSION_DEBUG_MODE == 1)
  input_ctx.data.debug = fusion_data.debug;
#endif
  g_sys_ui_data_status.is_fusion_data_ready_for_ui = true;

  // 2. Dust sensors
  if ((current_time_ms - input_ctx.last_dust_update_ms) >= SYS_INPUT_DUST_UPDATE_RATE_MS)
  {
    input_ctx.last_dust_update_ms = current_time_ms;
    sys_input_read_dust_sensor();
    g_sys_ui_data_status.is_dust_data_ready_for_ui = true;
  }

  // 3. Temperature and humidity sensors
  if ((current_time_ms - input_ctx.last_temp_hum_update_ms) >= SYS_INPUT_TEMP_HUM_UPDATE_RATE_MS)
  {
    input_ctx.last_temp_hum_update_ms = current_time_ms;
    if (bsp_temp_hum_read(&input_ctx.data.temp_hum) == STATUS_OK)
    {
      g_sys_ui_data_status.is_temp_hum_data_ready_for_ui = true;
    }
  }

// 4. Battery level
#if SYS_INPUT_BATT_ENABLE
  if (bsp_batt_is_initialized() && ((current_time_ms - input_ctx.batt_last_update_ms) >= SYS_INPUT_BATT_UPDATE_RATE_MS))
  {
    input_ctx.batt_last_update_ms = current_time_ms;
    sys_input_read_battery_level(&input_ctx.data.battery_level);
    g_sys_ui_data_status.is_battery_data_ready_for_ui = true;
  }
#else
// Do nothing
#endif

  // 5. Finalize
  input_ctx.data.timestamp_ms = current_time_ms;
  return STATUS_OK;
}

static void sys_input_process_idle(void)
{
  // TODO: Turn off sensors before sleeping
  OS_SEM_TAKE(sys_input_wakeup_sem, OS_MAX_DELAY);
  sys_manager_write_event(SYS_MANAGER_EVT_WAKEUP);
}

static void sys_input_process_locked(void)
{
  sys_fusion_danger_motion_flag_t flag = SYS_FUSION_DANGER_MOTION_NONE;
  sys_fusion_detect_danger_motion(&flag);
  switch (flag)
  {
  case SYS_FUSION_DANGER_MOTION_TILT:
  case SYS_FUSION_DANGER_MOTION_MOVING:
  case SYS_FUSION_DANGER_MOTION_VIBRATION:
  {
    sys_manager_write_event(SYS_MANAGER_EVT_DEVICE_DANGER);
    break;
  }
  default: break;
  }
}

void sys_input_wakeup_acc_handler(void)
{
  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE)
  {
    OS_SEM_GIVE_FROM_ISR(sys_input_wakeup_sem);
  }
}

/* End of file -------------------------------------------------------- */
