/**
 * @file       bsp_batt.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_batt.h"

#include "bsp_io.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"

#include <Wire.h>

#ifdef BATT_MONITOR_INA226
#include <INA226.h>
#elif defined(BATT_MONITOR_INA219)
#include <Adafruit_INA219.h>
#endif

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_batt, LOG_LEVEL_DBG);
#define INA226_I2C_ADDR      (0x40)
#define INA226_SHUNT_OHM     (0.100f)
#define INA226_MAX_CURRENT_A (8.192f)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
#ifdef BATT_MONITOR_INA226
static INA226 ina226(BATT_I2C_ADDR, &Wire1);
#elif defined(BATT_MONITOR_INA219)
static Adafruit_INA219 ina219(BATT_I2C_ADDR);
#endif

static bool is_initialized = false;

/* Private function prototypes ---------------------------------------- */
void bsp_i2c_scan(TwoWire *wire, const char *wire_name)
{
  LOG_INF("Scanning %s...", wire_name);
  int found = 0;

  for (uint8_t addr = 0x08; addr < 0x78; addr++)
  {
    wire->beginTransmission(addr);
    uint8_t err = wire->endTransmission();

    if (err == 0)
    {
      LOG_INF("Found device at 0x%02X", addr);
      found++;
    }
  }

  if (found == 0)
    LOG_WRN("No I2C devices found on %s", wire_name);
  else
    LOG_INF("Scan done. %d device(s) found", found);
}

/* Function definitions ----------------------------------------------- */
void bsp_batt_init(void)
{
  Wire1.begin(BATT_I2C_SDA_PIN, BATT_I2C_SCL_PIN);
  bsp_i2c_scan(&Wire1, "Wire1");

#ifdef BATT_MONITOR_INA226
  if (!ina226.begin())
  {
    LOG_ERR("INA226 not found at 0x%02X on Wire1", BATT_I2C_ADDR);
    is_initialized = false;
    return;
  }

  ina226.setMaxCurrentShunt(BATT_I2C_ADDR, BATT_I2C_ADDR);
#elif defined(BATT_MONITOR_INA219)
  if (!ina219.begin(&Wire1))
  {
    LOG_ERR("INA219 not found at 0x40 on Wire1");
    is_initialized = false;
    return;
  }
  ina219.setCalibration_32V_2A();
  LOG_DBG("INA219 initialized");

#endif
  is_initialized = true;
}

float bsp_batt_read_voltage_mv(void)
{
  if (!is_initialized)
  {
    LOG_WRN("BATT not initialized");
    return -1;
  }
  float voltage_mv = 0.0f;

#ifdef BATT_MONITOR_INA226
  voltage_mv = ina226.getBusVoltage() * 1000.0f;
#elif defined(BATT_MONITOR_INA219)
  voltage_mv = ina219.getBusVoltage_V() * 1000.0f;
#endif

  LOG_DBG("Voltage: %.2f mV", voltage_mv);
  return voltage_mv;
}

int32_t bsp_batt_read_current_ma(void)
{
  if (!is_initialized)
  {
    LOG_WRN("INA226 not initialized");
    return -1;
  }

  int32_t current_ma = 0;

#ifdef BATT_MONITOR_INA226
  float current_ua = ina226.getCurrent();
  current_ma       = (int32_t) (current_ua * 1000.0f);
#elif defined(BATT_MONITOR_INA219)
  current_ma = (int32_t) (ina219.getCurrent_mA());
#endif

  LOG_DBG("Current: %d mA", current_ma);
  return current_ma;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
