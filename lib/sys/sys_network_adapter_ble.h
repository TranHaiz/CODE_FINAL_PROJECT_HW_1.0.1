/**
 * @file       sys_network_adapter_ble.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      BLE network adapter — tethered relay via companion app.
 *             Uses Tracker Network Service (DATA/NOTI/CMD characteristics).
 *             bsp_ble Nordic UART service is left to sys_ble (log streaming).
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef SYS_NETWORK_ADAPTER_BLE_H
#define SYS_NETWORK_ADAPTER_BLE_H

/* Includes ----------------------------------------------------------- */
#include "sys_network_adapter.h"

/* Public variables --------------------------------------------------- */
extern net_adapter_t g_net_adapter_ble;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief BLE adapter task entry point — loops on poll().
 */
void sys_network_adapter_ble_task(void *param);

#endif /* SYS_NETWORK_ADAPTER_BLE_H */

/* End of file -------------------------------------------------------- */
