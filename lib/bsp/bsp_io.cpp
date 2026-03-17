/**
 * @file       bsp_io.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_io.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_io_init(int pin, bsp_io_mode_t mode)
{
  assert(pin >= 0);
  pinMode(pin, mode);
}

int bsp_io_read(int pin)
{
  assert(pin >= 0);
  return digitalRead(pin);
}

void bsp_io_write(int pin, bool value)
{
  assert(pin >= 0);
  digitalWrite(pin, value);
}

void bsp_io_int_init(int pin, bsp_io_event_t event, bsp_io_int_callback_t callback)
{
  assert(pin >= 0);
  assert(callback != NULL);

  attachInterrupt(digitalPinToInterrupt(pin), callback, event);
}
/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
