/**
 * @file       sys_network_adapter_ble.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      BLE network adapter implementation.
 *
 *             is_ready() == true requires:
 *               1. BLE link connected
 *               2. App writes handshake magic {0xA5, 0x01} to CMD char
 *
 *             publish() fragments payloads larger than BLE_FRAG_MAX_PAYLOAD
 *             using a 2-byte header per fragment:
 *               byte 0: seq (mod 256)
 *               byte 1: bit7=LAST, bit6=FIRST, bits5..0=channel (net_channel_t)
 *
 *             Inbound CMD writes from the app use the same framing and are
 *             reassembled before routing to incoming_cb.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_network_adapter_ble.h"

#include "bsp_ble.h"
#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_cmd.h"

#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(net_ble, LOG_LEVEL_SYS_BLE)

#define BLE_FRAG_MTU         (202)  // max bytes per BLE packet (header included)
#define BLE_FRAG_MAX_PAYLOAD (BLE_FRAG_MTU - 2)
#define BLE_REASSEMBLY_SIZE  (CMD_INPUT_MAX_LEN)
#define BLE_POLL_IDLE_MS     (20)

/* Handshake: app writes these 2 bytes to CMD char to signal relay-ready */
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

/* CMD reassembly state */
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
                                           status_function_t (*ble_send_function)(const uint8_t *, size_t));

/* Public adapter instance -------------------------------------------- */
net_adapter_t g_net_adapter_ble = {
  .name     = "BLE",
  .init     = ble_adapter_init,
  .is_ready = ble_adapter_is_ready,
  .publish  = ble_adapter_publish,
  .poll     = ble_adapter_poll,
};

/* Function definitions ----------------------------------------------- */
void sys_network_adapter_ble_task(void *param)
{
  while (1)
  {
    ble_adapter_poll();
  }
}

/* Private definitions ----------------------------------------------- */
static status_function_t ble_adapter_init(net_incoming_cb_t on_incoming)
{
  ble_incoming_cb = on_incoming;
  memset(&ble_ctx, 0, sizeof(ble_ctx));
  OS_SEM_CREATE(ble_adapter_cmd_sem);
  /* bsp_ble_init() already called by sys_ble_init() in setup() */
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

  status_function_t (*ble_send_function)(const uint8_t *, size_t);
  switch (ch)
  {
  case NET_CH_DATA: ble_send_function = bsp_ble_send_data; break;
  case NET_CH_NOTI: ble_send_function = bsp_ble_send_noti; break;
  case NET_CH_CMD_RESP: ble_send_function = bsp_ble_send_cmd_resp; break;
  default: return STATUS_ERROR;
  }

  return ble_fragment_send(ch, data, len, ble_send_function);
}

static void ble_adapter_poll(void)
{
  /* Block until a CMD write arrives (or timeout) */
  if (OS_SEM_TAKE(ble_adapter_cmd_sem, BLE_POLL_IDLE_MS) != pdTRUE)
  {
    return;
  }

  uint8_t cmd_buf[BLE_REASSEMBLY_SIZE];
  size_t  cmd_len = bsp_ble_get_cmd(cmd_buf, sizeof(cmd_buf));
  if (cmd_len == 0)
    return;

  if (!ble_ctx.connected)
    return;

  if (!ble_ctx.handshake_done)
  {
    /* Expect magic bytes to complete handshake */
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

  /* Reassemble fragmented command */
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
                                           status_function_t (*ble_send_function)(const uint8_t *, size_t))
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

    if (ble_send_function(frag, chunk + 2) != STATUS_OK)
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
