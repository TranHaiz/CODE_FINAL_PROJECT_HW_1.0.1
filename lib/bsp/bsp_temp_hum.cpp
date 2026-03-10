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

#include "log_service.h"


/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_temp_hum, LOG_LEVEL_DBG)

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
  // Initialize the SHT31 sensor
  if (Wire.begin(TEMP_HUM_I2C_SDA_PIN, TEMP_HUM_I2C_SCL_PIN))
    LOG_DBG("I2C bus initialized for Temp/Hum sensor");
  else
  {
    LOG_ERR("Failed to initialize I2C bus for Temp/Hum sensor");
    return STATUS_ERROR;
  }
  if (!temp_hum_handler.handler.begin(TEMP_HUM_I2C_ADDR))
  {
    LOG_ERR("Failed to initialize Temp/Hum sensor");
    return STATUS_ERROR;
  }
  temp_hum_handler.is_init = true;
  return STATUS_OK;
}

status_function_t bsp_temp_hum_read(temp_hum_data_t *data)
{
  if (!temp_hum_handler.is_init)
  {
    return STATUS_ERROR;
  }

  data->temperature = temp_hum_handler.handler.readTemperature();
  data->humidity    = temp_hum_handler.handler.readHumidity();

  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
