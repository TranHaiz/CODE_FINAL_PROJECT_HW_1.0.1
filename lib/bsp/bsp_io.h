/**
 * @file       bsp_io.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_IO_H_
#define _BSP_IO_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_IO_MODE_INPUT             = INPUT,
  BSP_IO_MODE_OUTPUT            = OUTPUT,
  BSP_IO_MODE_INPUT_PULLUP      = INPUT_PULLUP,
  BSP_IO_MODE_INPUT_PULLDOWN    = INPUT_PULLDOWN,
  BSP_IO_MODE_OPEN_DRAIN        = OPEN_DRAIN,
  BSP_IO_MODE_OUTPUT_OPEN_DRAIN = OUTPUT_OPEN_DRAIN,
  BSP_IO_MODE_ANALOG            = ANALOG
} bsp_io_mode_t;

typedef enum
{
  BSP_IO_EVENT_RISING  = RISING,
  BSP_IO_EVENT_FALLING = FALLING,
  BSP_IO_EVENT_CHANGE  = CHANGE,
  BSP_IO_EVENT_ONLOW   = ONLOW
} bsp_io_event_t;

typedef void (*bsp_io_int_callback_t)(void);

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void bsp_io_init(int pin, bsp_io_mode_t mode);
int  bsp_io_read(int pin);
void bsp_io_write(int pin, bool value);
void bsp_io_int_init(int pin, bsp_io_event_t event, bsp_io_int_callback_t callback);

#endif /*End file _BSP_IO_H_*/

/* End of file -------------------------------------------------------- */
