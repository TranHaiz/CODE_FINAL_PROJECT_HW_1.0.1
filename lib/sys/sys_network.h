/**
 * @file       sys_network.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      System Network Layer - Network management interface
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_NETWORK_H_
#define _SYS_NETWORK_H_

/* Includes ----------------------------------------------------------- */
#include "bsp_sim.h"
#include "common_type.h"
#include "sys_input.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern bool is_data_network_ready;

/* Public function prototypes ----------------------------------------- */
void sys_network_init(void);
void sys_network_process(void);

#endif /*End file _SYS_NETWORK_H_*/

/* End of file -------------------------------------------------------- */
