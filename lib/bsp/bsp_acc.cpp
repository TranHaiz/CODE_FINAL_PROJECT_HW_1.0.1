/**
 * @file       bsp_acc.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-05
 * @author     Hai Tran
 *
 * @brief      BSP implementation for LSM6DS3 Accelerometer/Gyroscope Sensor
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_acc.h"

#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"

#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <math.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_acc, LOG_LEVEL_ERROR);

#define LSM6DS3_REG_INT1_CTRL (0x0D)
#define LSM6DS3_REG_INT2_CTRL (0x0E)
#define LSM6DS3_REG_TAP_CFG0  (0x58)  // Tap config / global interrupt gate
#define LSM6DS3_REG_MD1_CFG   (0x5E)  // Source for INT1
#define LSM6DS3_REG_MD2_CFG   (0x5F)  // Source for INT2

#define LSM6DS3_BIT_DRDY_XL   (0x01)  // Accel data-ready
#define LSM6DS3_BIT_DRDY_G    (0x02)  // Gyro data-ready

#define LSM6DS3_BIT_MD_FF     (0x10)  // Free-fall
#define LSM6DS3_BIT_MD_WU     (0x20)  // Wake-up / motion
#define LSM6DS3_BIT_MD_DTAP   (0x08)  // Double-tap
#define LSM6DS3_BIT_MD_STAP   (0x40)  // Single-tap

#define LSM6DS3_BIT_INT_EN    (0x80)

/* Private enumerate/structure ---------------------------------------- */

/**
 * @brief Internal accelerometer context (singleton)
 */
typedef struct
{
  LSM6DS3             *sensor;
  bsp_acc_raw_data_t   raw_data;
  bsp_acc_data_t       processed_data;
  bsp_acc_int_source_t int1_source;   // Current INT1 event source
  bsp_acc_int_source_t int2_source;   // Current INT2 event source
  bool                 int1_enabled;  // INT1 enable flag
  bool                 int2_enabled;  // INT2 enable flag
  size_t               last_update_ms;
  bool                 is_initialized;
} bsp_acc_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_acc_ctx_t acc_handler = {
  .sensor         = nullptr,
  .raw_data       = { 0 },
  .processed_data = { 0 },
  .int1_source    = BSP_ACC_INT_ACCEL_DATA_READY,
  .int2_source    = BSP_ACC_INT_GYRO_DATA_READY,
  .int1_enabled   = false,
  .int2_enabled   = false,
  .last_update_ms = 0,
  .is_initialized = false,
};

/* Private function prototypes ---------------------------------------- */
static float bsp_acc_calculate_magnitude(float x, float y, float z);

/* Function definitions ----------------------------------------------- */
status_function_t bsp_acc_init(void)
{
  if (acc_handler.is_initialized)
  {
    return STATUS_OK;
  }

  Wire.begin(ACC_I2C_SDA_PIN, ACC_I2C_SCL_PIN);
  OS_DELAY_MS(100);

  acc_handler.sensor = new LSM6DS3(I2C_MODE, ACC_I2C_ADDR);
  if (acc_handler.sensor == nullptr)
  {
    LOG_ERR("Failed to create sensor object");
    return STATUS_ERROR;
  }

  acc_handler.sensor->settings.accelRange      = 2;
  acc_handler.sensor->settings.accelSampleRate = 104;
  acc_handler.sensor->settings.accelBandWidth  = 50;

  uint8_t result = acc_handler.sensor->begin();

  if (result != 0)
  {
    LOG_DBG("Failed with 0x%02X, trying 0x%02X...", ACC_I2C_ADDR, ACC_I2C_ADDR_ALT);
    delete acc_handler.sensor;

    acc_handler.sensor = new LSM6DS3(I2C_MODE, ACC_I2C_ADDR_ALT);
    if (acc_handler.sensor == nullptr)
    {
      LOG_ERR("Failed to create sensor object (alt addr)");
      return STATUS_ERROR;
    }

    acc_handler.sensor->settings.accelRange      = 2;
    acc_handler.sensor->settings.accelSampleRate = 104;
    acc_handler.sensor->settings.accelBandWidth  = 50;

    if (acc_handler.sensor->begin() != 0)
    {
      LOG_ERR("Failed to start sensor on both addresses");
      return STATUS_ERROR;
    }

    LOG_DBG("Connected on alternate address 0x%02X", ACC_I2C_ADDR_ALT);
  }

  memset(&acc_handler.raw_data, 0, sizeof(bsp_acc_raw_data_t));
  acc_handler.last_update_ms = OS_GET_TICK();
  acc_handler.is_initialized = true;

  LOG_INF("[ACC] Initialized: ±2g, 104Hz ODR, 50Hz BW");
  return STATUS_OK;
}
status_function_t bsp_acc_get_raw_data(bsp_acc_raw_data_t *data)
{
  if (data == nullptr || !acc_handler.is_initialized || acc_handler.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  // Read accelerometer data
  data->acc_x = acc_handler.sensor->readFloatAccelX();
  data->acc_y = acc_handler.sensor->readFloatAccelY();
  data->acc_z = acc_handler.sensor->readFloatAccelZ();

  // Read gyroscope data
  data->gyro_x = acc_handler.sensor->readFloatGyroX();
  data->gyro_y = acc_handler.sensor->readFloatGyroY();
  data->gyro_z = acc_handler.sensor->readFloatGyroZ();

  // Read temperature
  data->temp_c = acc_handler.sensor->readTempC();

  // Update internal cache
  acc_handler.raw_data       = *data;
  acc_handler.last_update_ms = OS_GET_TICK();

  return STATUS_OK;
}

status_function_t bsp_acc_get_data(bsp_acc_data_t *data)
{
  if (data == nullptr || !acc_handler.is_initialized)
  {
    return STATUS_ERROR;
  }

  // Calculate magnitude from current raw data
  float magnitude =
    bsp_acc_calculate_magnitude(acc_handler.raw_data.acc_x, acc_handler.raw_data.acc_y, acc_handler.raw_data.acc_z);

  acc_handler.processed_data.accel_magnitude = magnitude;
  acc_handler.processed_data.timestamp_ms    = acc_handler.last_update_ms;
  acc_handler.processed_data.is_valid        = true;

  *data = acc_handler.processed_data;
  return STATUS_OK;
}

status_function_t bsp_acc_config_interrupt(bsp_acc_int_pin_t pin, bsp_acc_int_source_t source)
{
  const uint8_t int_ctrl_reg = (pin == BSP_ACC_INT_PIN_1) ? LSM6DS3_REG_INT1_CTRL : LSM6DS3_REG_INT2_CTRL;
  const uint8_t md_cfg_reg   = (pin == BSP_ACC_INT_PIN_1) ? LSM6DS3_REG_MD1_CFG : LSM6DS3_REG_MD2_CFG;
  uint8_t       reg          = 0;

  if (!acc_handler.is_initialized || acc_handler.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  if (pin == BSP_ACC_INT_PIN_1)
  {
    acc_handler.int1_source = source;
  }
  else if (pin == BSP_ACC_INT_PIN_2)
  {
    acc_handler.int2_source = source;
  }
  else
  {
    return STATUS_ERROR;
  }

  switch (source)
  {
  case BSP_ACC_INT_ACCEL_DATA_READY:
    acc_handler.sensor->readRegister(&reg, int_ctrl_reg);
    acc_handler.sensor->writeRegister(int_ctrl_reg, reg | LSM6DS3_BIT_DRDY_XL);
    break;

  case BSP_ACC_INT_GYRO_DATA_READY:
    acc_handler.sensor->readRegister(&reg, int_ctrl_reg);
    acc_handler.sensor->writeRegister(int_ctrl_reg, reg | LSM6DS3_BIT_DRDY_G);
    break;

  case BSP_ACC_INT_MOTION_DETECT:
    acc_handler.sensor->readRegister(&reg, LSM6DS3_REG_TAP_CFG0);
    acc_handler.sensor->writeRegister(LSM6DS3_REG_TAP_CFG0, reg | (1 << 4) | (1 << 3) | (1 << 2) | LSM6DS3_BIT_INT_EN);
    acc_handler.sensor->writeRegister(0x5B, 0x02);
    acc_handler.sensor->readRegister(&reg, md_cfg_reg);
    acc_handler.sensor->writeRegister(md_cfg_reg, reg | LSM6DS3_BIT_MD_WU);
    break;

  case BSP_ACC_INT_SINGLE_TAP:
    acc_handler.sensor->readRegister(&reg, LSM6DS3_REG_TAP_CFG0);
    acc_handler.sensor->writeRegister(LSM6DS3_REG_TAP_CFG0, reg | (1 << 4) | (1 << 3) | (1 << 2) | LSM6DS3_BIT_INT_EN);
    acc_handler.sensor->writeRegister(0x57, 0x09);
    acc_handler.sensor->writeRegister(0x59, 0x06);
    acc_handler.sensor->readRegister(&reg, md_cfg_reg);
    acc_handler.sensor->writeRegister(md_cfg_reg, reg | LSM6DS3_BIT_MD_STAP);
    break;

  case BSP_ACC_INT_DOUBLE_TAP:
    acc_handler.sensor->readRegister(&reg, LSM6DS3_REG_TAP_CFG0);
    acc_handler.sensor->writeRegister(LSM6DS3_REG_TAP_CFG0, reg | (1 << 4) | (1 << 3) | (1 << 2) | LSM6DS3_BIT_INT_EN);
    acc_handler.sensor->writeRegister(0x57, 0x09);
    acc_handler.sensor->readRegister(&reg, 0x5C);
    acc_handler.sensor->writeRegister(0x5C, reg | 0x80);
    acc_handler.sensor->writeRegister(0x59, 0x7A);
    acc_handler.sensor->readRegister(&reg, md_cfg_reg);
    acc_handler.sensor->writeRegister(md_cfg_reg, reg | LSM6DS3_BIT_MD_DTAP);
    break;

  case BSP_ACC_INT_FREE_FALL:
    acc_handler.sensor->readRegister(&reg, LSM6DS3_REG_TAP_CFG0);
    acc_handler.sensor->writeRegister(LSM6DS3_REG_TAP_CFG0, reg | (1 << 4) | (1 << 3) | (1 << 2) | LSM6DS3_BIT_INT_EN);
    acc_handler.sensor->writeRegister(0x5D, (0x06 << 3) | 0x01);
    acc_handler.sensor->readRegister(&reg, md_cfg_reg);
    acc_handler.sensor->writeRegister(md_cfg_reg, reg | LSM6DS3_BIT_MD_FF);
    break;

  default: LOG_WRN("Unsupported interrupt source: %d", source); return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_acc_enable_interrupt(bsp_acc_int_pin_t pin)
{
  if (!acc_handler.is_initialized || acc_handler.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  const bsp_acc_int_source_t src = (pin == BSP_ACC_INT_PIN_1) ? acc_handler.int1_source : acc_handler.int2_source;
  if (pin == BSP_ACC_INT_PIN_1)
  {
    acc_handler.int1_enabled = true;
  }
  else if (pin == BSP_ACC_INT_PIN_2)
  {
    acc_handler.int2_enabled = true;
  }
  else
  {
    return STATUS_ERROR;
  }

  if (src != BSP_ACC_INT_ACCEL_DATA_READY && src != BSP_ACC_INT_GYRO_DATA_READY)
  {
    uint8_t reg = 0;
    acc_handler.sensor->readRegister(&reg, LSM6DS3_REG_TAP_CFG0);
    acc_handler.sensor->writeRegister(LSM6DS3_REG_TAP_CFG0, reg | LSM6DS3_BIT_INT_EN);
  }

  return STATUS_OK;
}

status_function_t bsp_acc_disable_interrupt(bsp_acc_int_pin_t pin)
{
  if (!acc_handler.is_initialized || acc_handler.sensor == nullptr)
  {
    return STATUS_ERROR;
  }

  if (pin != BSP_ACC_INT_PIN_1 && pin != BSP_ACC_INT_PIN_2)
  {
    return STATUS_ERROR;
  }

  const bsp_acc_int_source_t src      = (pin == BSP_ACC_INT_PIN_1) ? acc_handler.int1_source : acc_handler.int2_source;
  const uint8_t              int_ctrl = (pin == BSP_ACC_INT_PIN_1) ? LSM6DS3_REG_INT1_CTRL : LSM6DS3_REG_INT2_CTRL;
  const uint8_t              md_cfg   = (pin == BSP_ACC_INT_PIN_1) ? LSM6DS3_REG_MD1_CFG : LSM6DS3_REG_MD2_CFG;
  LSM6DS3                   *s        = acc_handler.sensor;
  uint8_t                    reg      = 0;

  switch (src)
  {
  case BSP_ACC_INT_ACCEL_DATA_READY:
    s->readRegister(&reg, int_ctrl);
    s->writeRegister(int_ctrl, reg & ~LSM6DS3_BIT_DRDY_XL);
    break;

  case BSP_ACC_INT_GYRO_DATA_READY:
    s->readRegister(&reg, int_ctrl);
    s->writeRegister(int_ctrl, reg & ~LSM6DS3_BIT_DRDY_G);
    break;

  case BSP_ACC_INT_MOTION_DETECT:
    s->readRegister(&reg, md_cfg);
    s->writeRegister(md_cfg, reg & ~LSM6DS3_BIT_MD_WU);
    break;

  case BSP_ACC_INT_SINGLE_TAP:
    s->readRegister(&reg, md_cfg);
    s->writeRegister(md_cfg, reg & ~LSM6DS3_BIT_MD_STAP);
    break;

  case BSP_ACC_INT_DOUBLE_TAP:
    s->readRegister(&reg, md_cfg);
    s->writeRegister(md_cfg, reg & ~LSM6DS3_BIT_MD_DTAP);
    break;

  case BSP_ACC_INT_FREE_FALL:
    s->readRegister(&reg, md_cfg);
    s->writeRegister(md_cfg, reg & ~LSM6DS3_BIT_MD_FF);
    break;

  default: break;
  }

  if (pin == BSP_ACC_INT_PIN_1)
  {
    acc_handler.int1_enabled = false;
  }
  else
  {
    acc_handler.int2_enabled = false;
  }

  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
