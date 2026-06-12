/**
 * @file       sys_cmd_usb.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-21
 * @author     Hai Tran
 *
 * @brief      System command through USB processing implementation
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_CMD_USB_H_
#define _SYS_CMD_USB_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_usb.h"
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_CMD_USB_CMD_RESET = 0,
  SYS_CMD_USB_CMD_UNLOCK,
  SYS_CMD_USB_CMD_LOCK,
  SYS_CMD_USB_CMD_SET_TIME,
  SYS_CMD_USB_CMD_SET_ID,
  SYS_CMD_USB_CMD_RESET_OFFLINE_DATA,
  SYS_CMD_USB_CMD_RESET_DISTANCE,
  SYS_CMD_USB_CMD_SET_DANGER_NOTI,
  SYS_CMD_USB_CMD_SLEEP,
  SYS_CMD_USB_CMD_FLUSH_FUSION_LOG,
  SYS_CMD_USB_CMD_DEVICE_INFO,
  SYS_CMD_USB_CMD_CHECK_LTE_BAND,
  SYS_CMD_USB_CMD_MODEM_RESET,
  SYS_CMD_USB_CMD_MAX
} sys_cmd_usb_cmd_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize USB command processing
 * @return none
 */
void sys_cmd_usb_init(void);

/**
 * @brief Process incoming USB command - call from a task loop
 * @return none
 */
void sys_cmd_usb_process(void);

#endif /*End file _SYS_CMD_USB_H_*/

/* End of file -------------------------------------------------------- */
