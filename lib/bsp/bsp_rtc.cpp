/**
 * @file       bsp_rtc.c
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2025-08-19
 * @author     Hai Tran
 *
 * @brief      Source file for RTC BSP layer.
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_rtc.h"

#include "device_config.h"
#include "log_service.h"

/* Private defines -----------------------------------------------------------*/
LOG_MODULE_REGISTER(bsp_rtc, LOG_LEVEL_DBG)

/* Private enumerate/structure -----------------------------------------------*/
typedef struct
{
  timeline_t time;
  RTC_DS3231 rtc;  // Using DS3231 RTC module
} rtc_handler_t;

/* Private macros ------------------------------------------------------------*/
/* Public variables --------------------------------------------------------- */
/* Private variables ---------------------------------------------------------*/
static rtc_handler_t rtc_handler;

/* Private function prototypes -----------------------------------------------*/

/* Function definitions ------------------------------------------------------*/
status_function_t bsp_rtc_init(void)
{
  Wire.begin(RTC_I2C_SDA_PIN, RTC_I2C_SCL_PIN);

  // 2. Kiểm tra kết nối RTC
  if (!rtc_handler.rtc.begin())
  {
    LOG_ERR("Failed to initialize RTC");
    return STATUS_ERROR;
  }
}

status_function_t bsp_rtc_set(timeline_t *time)
{
  DateTime dt(time->year, time->month, time->date, time->hour, time->minute, time->second);
  rtc_handler.rtc.adjust(dt);
}

status_function_t bsp_rtc_get(timeline_t *time)
{
  DateTime now = rtc_handler.rtc.now();
  time->second = now.second();
  time->minute = now.minute();
  time->hour   = now.hour();
  time->day    = (day_in_week_t) now.dayOfTheWeek();
  time->date   = now.day();
  time->month  = now.month();
  time->year   = now.year();
}

/* Private definitions ------------------------------------------------------ */

/* End of file ---------------------------------------------------------------*/
