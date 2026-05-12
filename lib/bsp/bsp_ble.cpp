/**
 * @file       bsp_ble.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      Board Support Package for Bluetooth Low Energy (BLE)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_ble.h"

#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_ble, LOG_LEVEL_BSP_BLE);

#define Nordic_UART_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define Nordic_UART_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define Nordic_UART_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define BSP_BLE_RX_BUFFER_SIZE             (512)
#define BSP_BLE_CMD_BUFFER_SIZE            (512)

#define TRACKER_SERVICE_UUID               "A0000001-0000-1000-8000-00805F9B34FB"
#define TRACKER_DATA_CHAR_UUID             "A0000002-0000-1000-8000-00805F9B34FB"
#define TRACKER_NOTI_CHAR_UUID             "A0000003-0000-1000-8000-00805F9B34FB"
#define TRACKER_CMD_CHAR_UUID              "A0000004-0000-1000-8000-00805F9B34FB"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_ble_cb_t ble_callback   = NULL;
static bsp_ble_cb_t ble_callback_2 = NULL;

static BLEServer         *pServer = NULL;
static BLECharacteristic *pTxCharacteristic;
static BLECharacteristic *pDataCharacteristic = NULL;
static BLECharacteristic *pNotiCharacteristic = NULL;
static BLECharacteristic *pCmdCharacteristic  = NULL;
static bool               deviceConnected     = false;

static uint8_t bsp_ble_rx_buffer[BSP_BLE_RX_BUFFER_SIZE];
static size_t  bsp_ble_rx_len = 0;

static uint8_t bsp_ble_cmd_buffer[BSP_BLE_CMD_BUFFER_SIZE];
static size_t  bsp_ble_cmd_len = 0;

/* Private function prototypes ---------------------------------------- */
static void bsp_ble_callback(bsp_ble_event_t event)
{
  if (ble_callback)
    ble_callback(event);
  if (ble_callback_2)
    ble_callback_2(event);
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    LOG_DBG("BLE connected");
    bsp_ble_callback(BSP_BLE_EVT_CONNECT);
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    LOG_DBG("BLE disconnected");
    bsp_ble_callback(BSP_BLE_EVT_DISCONNECT);
  }
};

class BleUartRxCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      bsp_ble_rx_len = rxValue.length();
      if (bsp_ble_rx_len > BSP_BLE_RX_BUFFER_SIZE)
        bsp_ble_rx_len = BSP_BLE_RX_BUFFER_SIZE;
      memcpy(bsp_ble_rx_buffer, rxValue.c_str(), bsp_ble_rx_len);
      LOG_DBG("BLE UART RX: %d bytes", bsp_ble_rx_len);
      bsp_ble_callback(BSP_BLE_EVT_RECEIVE_DATA);
    }
  }
};

class BleTrackerCmdCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string val = pCharacteristic->getValue();
    if (val.length() > 0)
    {
      bsp_ble_cmd_len = val.length();
      if (bsp_ble_cmd_len > BSP_BLE_CMD_BUFFER_SIZE)
        bsp_ble_cmd_len = BSP_BLE_CMD_BUFFER_SIZE;
      memcpy(bsp_ble_cmd_buffer, val.c_str(), bsp_ble_cmd_len);
      LOG_DBG("BLE CMD write: %d bytes", bsp_ble_cmd_len);
      bsp_ble_callback(BSP_BLE_EVT_CMD_WRITE);
    }
  }
};

/* Function definitions ----------------------------------------------- */
status_function_t bsp_ble_init(bsp_ble_cb_t cb)
{
  ble_callback = cb;

  // Set device name
  BLEDevice::init(g_device_info.device_name);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // --- Nordic UART service (general BLE: log streaming, error mode) ---
  BLEService *pUartService = pServer->createService(Nordic_UART_SERVICE_UUID);

  pTxCharacteristic =
    pUartService->createCharacteristic(Nordic_UART_CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic =
    pUartService->createCharacteristic(Nordic_UART_CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new BleUartRxCallbacks());

  pUartService->start();

  // --- Tracker Network service (sys_network_adapter_ble) ---
  BLEService *pTrackerService = pServer->createService(TRACKER_SERVICE_UUID);

  pDataCharacteristic =
    pTrackerService->createCharacteristic(TRACKER_DATA_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pDataCharacteristic->addDescriptor(new BLE2902());

  pNotiCharacteristic =
    pTrackerService->createCharacteristic(TRACKER_NOTI_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pNotiCharacteristic->addDescriptor(new BLE2902());

  pCmdCharacteristic = pTrackerService->createCharacteristic(
    TRACKER_CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCmdCharacteristic->addDescriptor(new BLE2902());
  pCmdCharacteristic->setCallbacks(new BleTrackerCmdCallbacks());

  pTrackerService->start();

  LOG_DBG("BLE init successful, pending advertise_start");
  return STATUS_OK;
}

status_function_t bsp_ble_advertise_start(void)
{
  if (pServer == NULL)
  {
    return STATUS_ERROR;
  }
  pServer->getAdvertising()->start();
  LOG_DBG("BLE advertising started");
  return STATUS_OK;
}

status_function_t bsp_ble_advertise_stop(void)
{
  if (pServer == NULL)
  {
    return STATUS_ERROR;
  }
  pServer->getAdvertising()->stop();
  LOG_DBG("BLE advertising stopped");
  return STATUS_OK;
}

status_function_t bsp_ble_send(const uint8_t *data, size_t size)
{
  if (!deviceConnected)
  {
    LOG_WRN("Cannot send, BLE disconnected");
    return STATUS_ERROR;
  }

  pTxCharacteristic->setValue((uint8_t *) data, size);
  pTxCharacteristic->notify();
  // LOG_DBG("BLE send %d bytes", size);

  return STATUS_OK;
}

status_function_t bsp_ble_disconnect(void)
{
  if (pServer == NULL)
  {
    return STATUS_ERROR;
  }

  // Gets the connection ID (assume 0 if one client) and disconnects
  if (deviceConnected)
  {
    pServer->disconnect(0);
    deviceConnected = false;
  }

  return STATUS_OK;
}

size_t bsp_ble_get(uint8_t *out_buffer, size_t max_size)
{
  if (out_buffer == NULL || max_size == 0 || bsp_ble_rx_len == 0)
  {
    return 0;
  }
  size_t copy_len = (bsp_ble_rx_len < max_size) ? bsp_ble_rx_len : max_size;
  memcpy(out_buffer, bsp_ble_rx_buffer, copy_len);
  bsp_ble_rx_len = 0;
  return copy_len;
}

status_function_t bsp_ble_add_callback(bsp_ble_cb_t cb)
{
  ble_callback_2 = cb;
  return STATUS_OK;
}

bool bsp_ble_is_connected(void)
{
  return deviceConnected;
}

status_function_t bsp_ble_send_data(const uint8_t *data, size_t size)
{
  if (!deviceConnected || pDataCharacteristic == NULL)
    return STATUS_ERROR;
  pDataCharacteristic->setValue((uint8_t *) data, size);
  pDataCharacteristic->notify();
  return STATUS_OK;
}

status_function_t bsp_ble_send_noti(const uint8_t *data, size_t size)
{
  if (!deviceConnected || pNotiCharacteristic == NULL)
    return STATUS_ERROR;
  pNotiCharacteristic->setValue((uint8_t *) data, size);
  pNotiCharacteristic->notify();
  return STATUS_OK;
}

status_function_t bsp_ble_send_cmd_resp(const uint8_t *data, size_t size)
{
  if (!deviceConnected || pCmdCharacteristic == NULL)
    return STATUS_ERROR;
  pCmdCharacteristic->setValue((uint8_t *) data, size);
  pCmdCharacteristic->notify();
  return STATUS_OK;
}

size_t bsp_ble_get_cmd(uint8_t *out_buffer, size_t max_size)
{
  if (out_buffer == NULL || max_size == 0 || bsp_ble_cmd_len == 0)
  {
    return 0;
  }
  size_t copy_len = (bsp_ble_cmd_len < max_size) ? bsp_ble_cmd_len : max_size;
  memcpy(out_buffer, bsp_ble_cmd_buffer, copy_len);
  bsp_ble_cmd_len = 0;
  return copy_len;
}

/* End of file -------------------------------------------------------- */
