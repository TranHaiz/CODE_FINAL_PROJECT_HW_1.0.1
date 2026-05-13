/**
 * @file       sys_network_adapter_ble.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      System service BLE
 *
 */

#ifndef SYS_NETWORK_ADAPTER_BLE_H
#define SYS_NETWORK_ADAPTER_BLE_H

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "sys_network_adapter.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern net_adapter_t g_net_adapter_ble;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the system BLE manager
 * @return none
 */
void sys_ble_init(void);

/**
 * @brief Process BLE events
 * @return none
 */
void sys_ble_process(void);

/**
 * @brief Send data over BLE (stored in TX static buffer, max 10 messages)
 * @param[in] data Pointer to the data to send
 * @param[in] len  Size of data
 * @return none
 */
void sys_ble_send(const uint8_t *data, size_t len);

/**
 * @brief Check if BLE is currently connected
 * @return true if connected, false otherwise
 */
bool sys_ble_is_connected(void);

/**
 * @brief Enable or disable BLE advertising.
 * @param[in] enable true to enable advertising, false to disable
 * @return none
 */
void sys_ble_set_advertise(bool enable);

#endif /* SYS_NETWORK_ADAPTER_BLE_H */

/* End of file -------------------------------------------------------- */
