/**
 * @file       sys_ble.cpp
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
#include "sys_ble.h"

#include "bsp_ble.h"
#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_manager.h"

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
#include "cbuffer.h"

OS_SEM_DEFINE_STATIC(sys_ble_rx_sem);
OS_SEM_DEFINE_STATIC(sys_ble_con_mutex);
OS_MUTEX_DEFINE_STATIC(sys_ble_tx_mutex);

static cbuffer_t     sys_ble_tx_cb;
static bool          sys_ble_connected       = false;
static bool          s_advertise_enabled     = false;
static uint8_t       sys_ble_buffer[SYS_BLE_MAX_RX_LEN];
static sys_ble_msg_t sys_ble_tx_buffer_array[SYS_BLE_TX_MAX_TX_MES];

/* Private function prototypes ---------------------------------------- */
static void sys_ble_callback_handler(bsp_ble_event_t event);

/* Function definitions ----------------------------------------------- */
void sys_ble_init(void)
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

void sys_ble_send(const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0 || len > SYS_BLE_MAX_TX_LEN)
  {
    return;
  }

  sys_ble_msg_t msg;
  msg.len = len;
  memcpy(msg.data, data, len);

  OS_MUTEX_LOCK(sys_ble_tx_mutex);
  // cbuffer ghi theo byte nên ta kiểm tra số byte thực tế ghi được so với struct msg
  if (cb_write(&sys_ble_tx_cb, &msg, sizeof(sys_ble_msg_t)) != sizeof(sys_ble_msg_t))
  {
    LOG_WRN("BLE TX buffer full, dropping message");
  }
  OS_MUTEX_UNLOCK(sys_ble_tx_mutex);
}

bool sys_ble_is_connected(void)
{
  return sys_ble_connected;
}

void sys_ble_set_advertise(bool enable)
{
  s_advertise_enabled = enable;
  if (enable)
  {
    bsp_ble_advertise_start();
  }
  else
  {
    bsp_ble_disconnect();
    bsp_ble_advertise_stop();
  }
}

void sys_ble_process(void)
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

  // 2. Process RX events
  if (OS_SEM_TAKE(sys_ble_rx_sem, 20) == true)
  {
    memset(sys_ble_buffer, 0, SYS_BLE_MAX_RX_LEN);
    size_t len = bsp_ble_get(sys_ble_buffer, sizeof(sys_ble_buffer) - 1);

    if (len > 0)
    {
      sys_ble_buffer[len] = '\0';
      LOG_INF("sys_ble got %d bytes: %s", len, (char *) sys_ble_buffer);
      // TODO: Handle some BLE commands from mobile app if needed
    }
  }
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

  case BSP_BLE_EVT_DISCONNECT:
    LOG_INF("BLE Client Disconnected");
    sys_ble_connected = false;
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

/* End of file -------------------------------------------------------- */
