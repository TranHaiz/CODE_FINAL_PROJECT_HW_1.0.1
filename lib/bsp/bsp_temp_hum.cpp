/**
 * @file       bsp_temp_hum.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief   Temperature and humidity sensor
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_temp_hum.h"

#include "Adafruit_SHT31.h"
#include "log_service.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_temp_hum, LOG_LEVEL_BSP_TEMP_HUM)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  Adafruit_SHT31 handler;
  bool           is_init;
} temp_hum_handler_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static temp_hum_handler_t temp_hum_handler = { .handler = Adafruit_SHT31(), .is_init = false };

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
status_function_t bsp_temp_hum_init(void)
{
  Wire.begin(TEMP_HUM_I2C_SDA_PIN, TEMP_HUM_I2C_SCL_PIN);

  if (!temp_hum_handler.handler.begin(TEMP_HUM_I2C_ADDR))
  {
    LOG_ERR("SHT3x not found at 0x%02X", TEMP_HUM_I2C_ADDR);
    return STATUS_ERROR;
  }

  temp_hum_handler.is_init = true;
  LOG_DBG("SHT3x initialized at 0x%02X", TEMP_HUM_I2C_ADDR);
  return STATUS_OK;
}

status_function_t bsp_temp_hum_read(temp_hum_data_t *data)
{
  if (!temp_hum_handler.is_init || data == nullptr)
  {
    return STATUS_ERROR;
  }

  float temp = NAN;
  float hum  = NAN;
  if (!temp_hum_handler.handler.readBoth(&temp, &hum))
  {
    LOG_ERR("SHT3x read failed (I2C error or CRC mismatch)");
    return STATUS_ERROR;
  }

  data->temperature = temp;
  data->humidity    = hum;
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
