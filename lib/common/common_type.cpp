/**
 * @file       common_type.c
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-12
 * @author     Hai Tran
 *
 * @brief      Common data type definitions
 *
 */

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
// clang-format off
#define INFO(i, name) [i] = name
char *FIREBASE_COMP_ID[FIREBASE_DATA_TYPE_MAX] = {
    INFO(BATTERY_LEVEL,        "Battery level"),
    INFO(GPS_POSITION,         "Position"),
    INFO(SPEED,                "Speed")
};
// clang-format on

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
/* Private definitions ----------------------------------------------- */

/* End of file ------------------------------------------------------- */
