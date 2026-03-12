/**
 * @file       common_type.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Common type definitions used across the project
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _COMMON_TYPE_H_
#define _COMMON_TYPE_H_
/* Includes ----------------------------------------------------------- */
#include "assert.h"
#include "esp_check.h"

#include <Arduino.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  STATUS_OK = 0,
  STATUS_ERROR,
  STATUS_BUSY,
  STATUS_TIMEOUT
} status_function_t;

typedef enum
{
  BATTERY_LEVEL = 0,
  GPS_POSITION,
  SPEED,
  FIREBASE_DATA_TYPE_MAX
} firebase_data_type_t;

typedef struct
{
  float latitude;
  float longitude;
} gps_position_type_t;

typedef struct
{
  float temperature;
  float humidity;
} temp_hum_data_t;

typedef enum
{
  SUNDAY = 1,
  MONDAY,
  TUESDAY,
  WEDNESDAY,
  THURSDAY,
  FRIDAY,
  SATURDAY
} day_in_week_t;
typedef struct
{
  uint8_t       second;  // Seconds [0-59]
  uint8_t       minute;  // Minutes [0-59]
  uint8_t       hour;    // Hours [0-23]
  day_in_week_t day;     // Day of week [1-7]
  uint8_t       date;    // Day of month [1-31]
  uint8_t       month;   // Month [1-12]
  uint32_t      year;    // Full year (e.g. 2025)
} timeline_t;

typedef struct
{
  uint8_t             batt_level;
  float               speed;
  gps_position_type_t position;
} firebase_data_t;

/* Public macros ------------------------------------------------------ */
#define STRING2NUMBER(x)   #x
#define assert_param(expr) assert(expr)

/* Public variables --------------------------------------------------- */
extern char *FIREBASE_COMP_ID[FIREBASE_DATA_TYPE_MAX];

/* Public function prototypes ----------------------------------------- */

#endif /*End file _COMMON_TYPE_H_*/

/* End of file -------------------------------------------------------- */
