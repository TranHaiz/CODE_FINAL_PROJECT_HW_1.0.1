/**
 * @file       sys_network_adapter_ble.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      System service BLE
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_network_adapter_ble.h"

#include "bsp_ble.h"
#include "cbuffer.h"
#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_cmd.h"

#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_ble, LOG_LEVEL_SYS_BLE);

#define SYS_BLE_MAX_RX_LEN    (512)
#define SYS_BLE_MAX_TX_LEN    (512)
#define SYS_BLE_TX_MAX_TX_MES (10)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  uint8_t data[SYS_BLE_MAX_TX_LEN];
  size_t  len;
} sys_ble_msg_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
OS_SEM_DEFINE_STATIC(sys_ble_rx_sem);
OS_SEM_DEFINE_STATIC(sys_ble_con_mutex);
OS_MUTEX_DEFINE_STATIC(sys_ble_tx_mutex);

static cbuffer_t     sys_ble_tx_cb;
static bool          sys_ble_connected   = false;
static bool          s_advertise_enabled = false;
static uint8_t       sys_ble_buffer[SYS_BLE_MAX_RX_LEN];
static sys_ble_msg_t sys_ble_tx_buffer_array[SYS_BLE_TX_MAX_TX_MES];

/* Private function prototypes ---------------------------------------- */
static void sys_ble_callback_handler(bsp_ble_event_t event);
static void ble_adapter_poll(void);

/* Function definitions ----------------------------------------------- */
void sys_network_adapter_ble_init(void)
{
  OS_SEM_CREATE(sys_ble_rx_sem);
  OS_MUTEX_CREATE(sys_ble_tx_mutex);
  OS_SEM_CREATE(sys_ble_con_mutex);
  cb_init(&sys_ble_tx_cb, sys_ble_tx_buffer_array, sizeof(sys_ble_tx_buffer_array));

  if (bsp_ble_init(sys_ble_callback_handler) != STATUS_OK)
  {
    LOG_ERR("Failed to initialize BSP BLE");
    return;
  }
#if (!DEVICE_BLE_FALLBACK_ENABLED)
  s_advertise_enabled = true;
  bsp_ble_advertise_start();
#endif
}

void sys_network_adapter_ble_send(const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0 || len > SYS_BLE_MAX_TX_LEN)
  {
    return;
  }

  sys_ble_msg_t msg;
  msg.len = len;
  memcpy(msg.data, data, len);

  OS_MUTEX_LOCK(sys_ble_tx_mutex);
  if (cb_write(&sys_ble_tx_cb, &msg, sizeof(sys_ble_msg_t)) != sizeof(sys_ble_msg_t))
  {
    LOG_WRN("BLE TX buffer full, dropping message");
  }
  OS_MUTEX_UNLOCK(sys_ble_tx_mutex);
}

bool sys_network_adapter_ble_is_connected(void)
{
  return sys_ble_connected;
}

void sys_network_adapter_ble_set_advertise(bool enable)
{
  if (s_advertise_enabled == enable)
  {
    return;
  }

  if (enable)
  {
    bsp_ble_advertise_start();
  }
  else
  {
    bsp_ble_disconnect();
    bsp_ble_advertise_stop();
  }
  s_advertise_enabled = enable;
}

void sys_network_adapter_ble_process(void)
{
  if (!sys_ble_connected)
  {
    // Block task permanently until BLE connects
    OS_SEM_TAKE(sys_ble_con_mutex, OS_MAX_DELAY);
  }

  // 1. Process TX queue
  if (sys_ble_connected)
  {
    sys_ble_msg_t tx_msg;
    OS_MUTEX_LOCK(sys_ble_tx_mutex);
    size_t read_bytes = cb_read(&sys_ble_tx_cb, &tx_msg, sizeof(sys_ble_msg_t));
    OS_MUTEX_UNLOCK(sys_ble_tx_mutex);

    if (read_bytes == sizeof(sys_ble_msg_t))
    {
      bsp_ble_send(tx_msg.data, tx_msg.len);
    }
  }

  // 2. Process RX events (non-blocking, adapter poll controls timing)
  if (OS_SEM_TAKE(sys_ble_rx_sem, 0) == true)
  {
    memset(sys_ble_buffer, 0, SYS_BLE_MAX_RX_LEN);
    size_t len = bsp_ble_get(sys_ble_buffer, sizeof(sys_ble_buffer) - 1);

    if (len > 0)
    {
      sys_ble_buffer[len] = '\0';
      LOG_INF("sys_ble got %d bytes: %s", len, (char *) sys_ble_buffer);
    }
  }

  // 3. BLE adapter poll — handles Tracker CMD writes, handshake, reassembly
  ble_adapter_poll();
}

/* Private definitions ----------------------------------------------- */
static void sys_ble_callback_handler(bsp_ble_event_t event)
{
  switch (event)
  {
  case BSP_BLE_EVT_CONNECT:
    LOG_INF("BLE Client Connected - Stop advertising");
    sys_ble_connected = true;
    OS_SEM_GIVE(sys_ble_con_mutex);
    bsp_ble_advertise_stop();  // Optional: Usually stop advertising when 1 client connected
    break;

  case BSP_BLE_EVT_DISCONNECT: LOG_INF("BLE Client Disconnected"); sys_ble_connected = false;
#if (DEVICE_BLE_FALLBACK_ENABLED)
    if (s_advertise_enabled)
      bsp_ble_advertise_start();
#else
    bsp_ble_advertise_start();
#endif
    break;

  case BSP_BLE_EVT_RECEIVE_DATA: OS_SEM_GIVE(sys_ble_rx_sem); break;

  default: break;
  }
}

/* Private defines ---------------------------------------------------- */
#define BLE_FRAG_MTU         (202)
#define BLE_FRAG_MAX_PAYLOAD (BLE_FRAG_MTU - 2)
#define BLE_REASSEMBLY_SIZE  (CMD_INPUT_MAX_LEN)
#define BLE_POLL_IDLE_MS     (20)

#define BLE_HS_MAGIC_0       (0xA5)
#define BLE_HS_MAGIC_1       (0x01)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  bool    connected;
  bool    handshake_done;
  uint8_t frag_seq;
} ble_adapter_ctx_t;

/* Private variables -------------------------------------------------- */
static ble_adapter_ctx_t ble_ctx;
static net_incoming_cb_t ble_incoming_cb = NULL;

static uint8_t ble_reassembly_buf[BLE_REASSEMBLY_SIZE];
static size_t  ble_reassembly_len    = 0;
static uint8_t ble_expected_seq      = 0;
static bool    ble_reassembly_active = false;

OS_SEM_DEFINE_STATIC(ble_adapter_cmd_sem);

/* Private function prototypes ---------------------------------------- */
static status_function_t ble_adapter_init(net_incoming_cb_t on_incoming);
static bool              ble_adapter_is_ready(void);
static status_function_t ble_adapter_publish(net_channel_t ch, const uint8_t *data, size_t len);
static void              ble_adapter_poll(void);
static void              ble_adapter_bsp_cb(bsp_ble_event_t event);
static void              ble_adapter_handle_cmd(const uint8_t *data, size_t len);
static status_function_t ble_fragment_send(net_channel_t  ch,
                                           const uint8_t *data,
                                           size_t         len,
                                           status_function_t (*send_fn)(const uint8_t *, size_t));

/* Public adapter instance -------------------------------------------- */
net_adapter_t g_net_adapter_ble = {
  .name     = "BLE",
  .init     = ble_adapter_init,
  .is_ready = ble_adapter_is_ready,
  .publish  = ble_adapter_publish,
  .poll     = ble_adapter_poll,
};

/* Private definitions ----------------------------------------------- */
static status_function_t ble_adapter_init(net_incoming_cb_t on_incoming)
{
  ble_incoming_cb = on_incoming;
  memset(&ble_ctx, 0, sizeof(ble_ctx));
  OS_SEM_CREATE(ble_adapter_cmd_sem);
  bsp_ble_add_callback(ble_adapter_bsp_cb);
  return STATUS_OK;
}

static bool ble_adapter_is_ready(void)
{
  return ble_ctx.connected && ble_ctx.handshake_done;
}

static status_function_t ble_adapter_publish(net_channel_t ch, const uint8_t *data, size_t len)
{
  if (!ble_adapter_is_ready() || data == NULL || len == 0)
    return STATUS_ERROR;

  status_function_t (*send_fn)(const uint8_t *, size_t);
  switch (ch)
  {
  case NET_CH_DATA: send_fn = bsp_ble_send_data; break;
  case NET_CH_NOTI: send_fn = bsp_ble_send_noti; break;
  case NET_CH_CMD_RESP: send_fn = bsp_ble_send_cmd_resp; break;
  default: return STATUS_ERROR;
  }

  return ble_fragment_send(ch, data, len, send_fn);
}

static void ble_adapter_poll(void)
{
  if (OS_SEM_TAKE(ble_adapter_cmd_sem, BLE_POLL_IDLE_MS) != pdTRUE)
    return;

  uint8_t cmd_buf[BLE_REASSEMBLY_SIZE];
  size_t  cmd_len = bsp_ble_get_cmd(cmd_buf, sizeof(cmd_buf));
  if (cmd_len == 0)
    return;

  if (!ble_ctx.connected)
    return;

  if (!ble_ctx.handshake_done)
  {
    if (cmd_len == 2 && cmd_buf[0] == BLE_HS_MAGIC_0 && cmd_buf[1] == BLE_HS_MAGIC_1)
    {
      ble_ctx.handshake_done = true;
      LOG_INF("BLE handshake OK — relay ready");
    }
    else
    {
      LOG_WRN("BLE: unexpected pre-handshake write (%d bytes)", (int) cmd_len);
    }
    return;
  }

  ble_adapter_handle_cmd(cmd_buf, cmd_len);
}

static void ble_adapter_bsp_cb(bsp_ble_event_t event)
{
  switch (event)
  {
  case BSP_BLE_EVT_CONNECT:
    ble_ctx.connected      = true;
    ble_ctx.handshake_done = false;
    LOG_INF("BLE adapter: connected");
    break;
  case BSP_BLE_EVT_DISCONNECT:
    ble_ctx.connected      = false;
    ble_ctx.handshake_done = false;
    ble_reassembly_active  = false;
    ble_reassembly_len     = 0;
    LOG_INF("BLE adapter: disconnected");
    break;
  case BSP_BLE_EVT_CMD_WRITE: OS_SEM_GIVE(ble_adapter_cmd_sem); break;
  default: break;
  }
}

static void ble_adapter_handle_cmd(const uint8_t *data, size_t len)
{
  if (len < 2)
    return;

  uint8_t        seq      = data[0];
  uint8_t        flags    = data[1];
  bool           is_first = (flags & 0x40) != 0;
  bool           is_last  = (flags & 0x80) != 0;
  const uint8_t *payload  = data + 2;
  size_t         pl_len   = len - 2;

  if (is_first)
  {
    ble_reassembly_len    = 0;
    ble_reassembly_active = true;
    ble_expected_seq      = (uint8_t) (seq + 1);
  }
  else if (!ble_reassembly_active || seq != ble_expected_seq)
  {
    LOG_WRN("BLE CMD: sequence error — discarding (expected %d got %d)", ble_expected_seq, seq);
    ble_reassembly_active = false;
    ble_reassembly_len    = 0;
    return;
  }
  else
  {
    ble_expected_seq = (uint8_t) (seq + 1);
  }

  if ((ble_reassembly_len + pl_len) <= sizeof(ble_reassembly_buf))
  {
    memcpy(ble_reassembly_buf + ble_reassembly_len, payload, pl_len);
    ble_reassembly_len += pl_len;
  }
  else
  {
    LOG_WRN("BLE CMD: reassembly buffer overflow — discarding");
    ble_reassembly_active = false;
    ble_reassembly_len    = 0;
    return;
  }

  if (is_last)
  {
    ble_reassembly_active = false;
    if (ble_incoming_cb != NULL && ble_reassembly_len > 0)
    {
      ble_incoming_cb(ble_reassembly_buf, ble_reassembly_len);
    }
    ble_reassembly_len = 0;
  }
}

static status_function_t ble_fragment_send(net_channel_t  ch,
                                           const uint8_t *data,
                                           size_t         len,
                                           status_function_t (*send_fn)(const uint8_t *, size_t))
{
  size_t  offset = 0;
  uint8_t seq    = ble_ctx.frag_seq;
  uint8_t frag[BLE_FRAG_MTU];

  while (offset < len)
  {
    size_t chunk    = (len - offset > BLE_FRAG_MAX_PAYLOAD) ? BLE_FRAG_MAX_PAYLOAD : (len - offset);
    bool   is_first = (offset == 0);
    bool   is_last  = (offset + chunk >= len);

    frag[0] = seq++;
    frag[1] = (uint8_t) ((is_last ? 0x80u : 0u) | (is_first ? 0x40u : 0u) | ((uint8_t) ch & 0x3Fu));
    memcpy(frag + 2, data + offset, chunk);

    if (send_fn(frag, chunk + 2) != STATUS_OK)
    {
      LOG_WRN("BLE fragment send failed at offset %u", (unsigned) offset);
      return STATUS_ERROR;
    }
    offset += chunk;
  }

  ble_ctx.frag_seq = seq;
  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
