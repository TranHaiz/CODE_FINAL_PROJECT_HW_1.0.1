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
