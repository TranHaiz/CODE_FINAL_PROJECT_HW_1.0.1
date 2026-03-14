/**
 * @file       sys_network.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      System Network Layer - Implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_network.h"

#include "log_service.h"


/* Private defines ---------------------------------------------------- */

#define MQTT_CLIENT_ID       "haq-trk-001"
#define MQTT_TOPIC_TELEMETRY "haq-trk-001/data"
#define MQTT_TOPIC_COMMAND   "haq-trk-001/cmd"
#define MQTT_KEEPALIVE_S     (60)
#define MQTT_QOS             (1)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

void sys_network_init(void)
{
}

void sys_network_process(void)
{
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
