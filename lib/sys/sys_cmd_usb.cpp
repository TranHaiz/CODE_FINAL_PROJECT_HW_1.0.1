/**
 * @file       sys_cmd_usb.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System command through USB processing implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_cmd_usb.h"

#include "log_service.h"
#include "os_lib.h"
#include "sys_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_cmd_usb, LOG_LEVEL_DBG);

#define CMD_USB_MAX_LEN (128)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  char command[32];
  void (*handler)(void);
} sys_cmd_usb_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void              sys_cmd_usb_callback_handler(bsp_usb_event_t event, void *arg);
static status_function_t sys_cmd_usb_parse_and_execute(const char *input);
static void              sys_cmd_usb_reset_handler(void);

/* Private variables -------------------------------------------------- */
// clang-format off
#define INFO(name, handler) { name, handler }
static sys_cmd_usb_t CMD_USB_INFO[SYS_CMD_USB_CMD_MAX] = {
  INFO("RESET",      sys_cmd_usb_reset_handler),
};
#undef INFO
// clang-format on

OS_SEM_DECLARE(sys_cmd_usb_req_sem)

/* Function definitions ----------------------------------------------- */
void sys_cmd_usb_init(void)
{
  bsp_usb_init(sys_cmd_usb_callback_handler);
  OS_SEM_CREATE(sys_cmd_usb_req_sem);
}

void sys_cmd_usb_process(void)
{
  OS_SEM_TAKE(sys_cmd_usb_req_sem, OS_MAX_DELAY);
  char cmd_buffer[CMD_USB_MAX_LEN];
  bsp_usb_read((uint8_t *) cmd_buffer, CMD_USB_MAX_LEN);
  if (sys_cmd_usb_parse_and_execute(cmd_buffer) == STATUS_OK)
  {
    LOG_DBG("Command executed successfully");
  }
  else
  {
    LOG_WRN("Failed to execute command: %s", cmd_buffer);
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_cmd_usb_callback_handler(bsp_usb_event_t event, void *arg)
{
  switch (event)
  {
  case BSP_USB_EVENT_DATA_RX:
  {
    OS_SEM_GIVE(sys_cmd_usb_req_sem);
    break;
  }
  case BSP_USB_EVENT_CONNECTED:
  case BSP_USB_EVENT_DISCONNECTED:
  {
    // Do nothing for now
    break;
  }

  default: break;
  }
}

static status_function_t sys_cmd_usb_parse_and_execute(const char *input)
{
  LOG_DBG("Processing command: %s", input);
  for (int i = 0; i < SYS_CMD_USB_CMD_MAX; ++i)
  {
    if (strncmp(input, CMD_USB_INFO[i].command, strlen(CMD_USB_INFO[i].command)) == 0)
    {
      if (CMD_USB_INFO[i].handler)
      {
        CMD_USB_INFO[i].handler();
        return STATUS_OK;
      }
      else
      {
        // Do nothing
        return STATUS_OK;
      }
    }
  }
  LOG_WRN("Unknown command: %s", input);
  return STATUS_ERROR;
}

static void sys_cmd_usb_reset_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_REBOOT);
}

/* End of file -------------------------------------------------------- */
