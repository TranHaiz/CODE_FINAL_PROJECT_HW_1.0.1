
/**
 * @file       sys_cmd.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief    System command processing header file
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_CMD_H_
#define _SYS_CMD_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "os_lib.h"

/* Public defines ----------------------------------------------------- */
#define CMD_INPUT_MAX_LEN (128)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  CMD_LOCK_DEVICE = 0,
  CMD_UNLOCK_DEVICE,
  CMD_SET_TIME,
  CMD_SET_DEVICE,
  CMD_REBOOT,
  CMD_STOP_RENTAL_FAIL,
  CMD_STOP_RENTAL_SUCCESS,
  CMD_SET_DANGER_NOTI,
  CMD_MAX
} sys_command_id_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
OS_SEM_DECLARE(sys_cmd_req_sem)
extern char g_cmd_input_buffer[CMD_INPUT_MAX_LEN];

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Process incoming command - call from a task loop
 * @return none
 */
void sys_cmd_process(void);

#endif /*End file _SYS_CMD_H_*/

/* End of file -------------------------------------------------------- */
