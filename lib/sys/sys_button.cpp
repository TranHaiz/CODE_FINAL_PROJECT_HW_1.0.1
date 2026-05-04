/**
 * @file       sys_button.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-04-28
 * @author     Hai Tran
 *
 * @brief      System Button Layer implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_button.h"

#include "cbuffer.h"
#include "log_service.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_button, LOG_LEVEL_SYS_BUTTON)
#define SYS_BUTTON_MAX_EVENTS 10

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
OS_SEM_DEFINE_STATIC(sys_button);
static cbuffer_t               s_sys_button_cb;
static bsp_button_press_type_t s_sys_button_cb_buf[SYS_BUTTON_MAX_EVENTS + 1];

/* Private function prototypes ---------------------------------------- */
static void sys_button_callback(bsp_button_press_type_t press_type);

/* Function definitions ----------------------------------------------- */

void sys_button_init(void)
{
  OS_SEM_CREATE(sys_button);

  cb_init(&s_sys_button_cb, s_sys_button_cb_buf, sizeof(s_sys_button_cb_buf));
  cb_clear(&s_sys_button_cb);

  bsp_button_init(BUTTON_EVT, sys_button_callback);
}

void sys_button_process(void)
{
  bsp_button_press_type_t event;

  bsp_button_process();

  if (OS_SEM_TAKE(sys_button, 20) == pdTRUE)
  {
    while (cb_read(&s_sys_button_cb, &event, sizeof(event)) == sizeof(event))
    {
      switch (event)
      {
      case BUTTON_PRESS_SHORT:
      {
        LOG_DBG("Button short press detected");
        break;
      }

      case BUTTON_PRESS_LONG:
      {
        LOG_DBG("Button long press detected");
        break;
      }
      case BUTTON_PRESS_DOUBLE:
      {
        LOG_DBG("Button double press detected");
        break;
      }
      default: break;
      }
    }
  }
}

/* Private definitions ------------------------------------------------ */

static void sys_button_callback(bsp_button_press_type_t press_type)
{
  /* Push event to the cbuffer so process task can handle it */
  if (cb_space_count(&s_sys_button_cb) >= sizeof(press_type))
  {
    cb_write(&s_sys_button_cb, &press_type, sizeof(press_type));
  }

  /* Wake up the process loop */
  OS_SEM_GIVE(sys_button);
}

/* End of file -------------------------------------------------------- */