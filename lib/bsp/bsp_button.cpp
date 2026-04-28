/**
 * @file       bsp_button.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-28
 * @author     Hai Tran
 *
 * @brief      Button control interface implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_button.h"

#include "bsp_io.h"
#include "device_config.h"
#include "os_lib.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  uint8_t               pin;
  bsp_button_callback_t cb;
  uint32_t              press_time;
  uint32_t              release_time;
  bool                  is_pressed;
  bool                  is_long_handled;
  uint8_t               click_count;
  uint32_t              last_click_time;
} bsp_button_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_button_ctx_t s_buttons[BUTTON_MAX];

/* Private function prototypes ---------------------------------------- */
static void IRAM_ATTR button_evt_isr_handler(void);

/* Function definitions ----------------------------------------------- */

void bsp_button_init(bsp_button_type_t button, bsp_button_callback_t callback)
{
  if (button >= BUTTON_MAX)
  {
    return;
  }

  s_buttons[button].pin             = IO_BUTTON_PIN;
  s_buttons[button].cb              = callback;
  s_buttons[button].press_time      = 0;
  s_buttons[button].release_time    = 0;
  s_buttons[button].is_pressed      = false;
  s_buttons[button].is_long_handled = false;
  s_buttons[button].click_count     = 0;
  s_buttons[button].last_click_time = 0;

  bsp_io_init(s_buttons[button].pin, BSP_IO_MODE_INPUT_PULLUP);
  if (button == BUTTON_EVT)
  {
    bsp_io_int_init(s_buttons[button].pin, BSP_IO_EVENT_CHANGE, button_evt_isr_handler);
  }
}

void bsp_button_process(void)
{
  for (int i = 0; i < BUTTON_MAX; i++)
  {
    uint32_t          now = OS_GET_TICK();
    bsp_button_ctx_t *btn = &s_buttons[i];

    if (btn->is_pressed && !btn->is_long_handled)
    {
      if ((now - btn->press_time) >= BUTTON_LONG_PRESS_MS)
      {
        btn->is_long_handled = true;
        btn->click_count     = 0;

        if (btn->cb)
        {
          btn->cb(BUTTON_PRESS_LONG);
        }
      }
    }

    if (!btn->is_pressed && btn->click_count > 0)
    {
      if ((now - btn->last_click_time) > BUTTON_DOUBLE_PRESS_MS)
      {
        if (btn->cb)
        {
          if (btn->click_count == 1)
          {
            btn->cb(BUTTON_PRESS_SHORT);
          }
          else if (btn->click_count >= 2)
          {
            btn->cb(BUTTON_PRESS_DOUBLE);
          }
        }
        btn->click_count = 0;
      }
    }
  }
}

/* Private definitions ------------------------------------------------ */
static void IRAM_ATTR button_evt_isr_handler(void)
{
  bsp_button_ctx_t *btn = &s_buttons[BUTTON_EVT];
  uint32_t          now = OS_GET_TICK();

  bool current_state = (bsp_io_read(btn->pin) == 0);

  if (current_state && !btn->is_pressed)
  {
    if ((now - btn->release_time) > BUTTON_DEBOUNCE_MS)
    {
      btn->is_pressed      = true;
      btn->press_time      = now;
      btn->is_long_handled = false;
    }
  }
  else if (!current_state && btn->is_pressed)
  {
    if ((now - btn->press_time) > BUTTON_DEBOUNCE_MS)
    {
      btn->is_pressed   = false;
      btn->release_time = now;

      uint32_t press_duration = now - btn->press_time;

      if (!btn->is_long_handled && press_duration >= BUTTON_DEBOUNCE_MS)
      {
        btn->click_count++;
        btn->last_click_time = now;
      }
    }
  }
}

/* End of file -------------------------------------------------------- */