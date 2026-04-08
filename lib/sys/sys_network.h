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
/**
 * @brief Initialize the network layer, including SIM and MQTT setup.
 *
 * @return none
 */
void sys_network_init(void);

/**
 * @brief Process the network state machine. Should be called periodically.
 *
 * @param[in] param  Unused.
 *
 * @return none
 */
void sys_network_process(void *param);

/**
 * @brief Data task entry point. Register as a FreeRTOS task.
 *        Pulls telemetry from sys_input and stores into cbuffer.
 *        If cbuffer usage exceeds 80%, flushes data to SD.
 *
 * @param[in] param  Unused.
 *
 * @return none
 */
void sys_network_data_task(void *param);

/**
 * @brief Trigger a network wakeup (e.g. after device unlock)
 *
 * @return none
 */
void sys_network_wakeup(void);

#endif /*End file _SYS_NETWORK_H_*/

/* End of file -------------------------------------------------------- */
