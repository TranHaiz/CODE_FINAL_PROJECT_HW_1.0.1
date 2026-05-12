/**
 * @file       sys_network_adapter.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      Common interface for network transport adapters (LTE, BLE, …).
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef SYS_NETWORK_ADAPTER_H
#define SYS_NETWORK_ADAPTER_H

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <stddef.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  NET_CH_DATA = 0,  // telemetry stream (high volume)
  NET_CH_NOTI,      // alarm / status events
  NET_CH_CMD_RESP,  // outbound command responses
  NET_CH_MAX
} net_channel_t;

/* Called by an adapter when an inbound command arrives */
typedef void (*net_incoming_cb_t)(const uint8_t *data, size_t len);

typedef struct
{
  const char       *name;
  /* Register incoming-command callback and set up hardware/resources. */
  status_function_t (*init)(net_incoming_cb_t on_incoming);
  /* True when the adapter is connected and ready to publish. */
  bool (*is_ready)(void);
  /* Send data on the given channel. Returns STATUS_OK or STATUS_ERROR. */
  status_function_t (*publish)(net_channel_t ch, const uint8_t *data, size_t len);
  /* Advance the adapter's internal FSM — called from the adapter's own task. */
  void (*poll)(void);
} net_adapter_t;

#endif /* SYS_NETWORK_ADAPTER_H */

/* End of file -------------------------------------------------------- */
