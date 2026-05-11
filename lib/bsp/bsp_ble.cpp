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

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_ble_cb_t ble_callback = NULL;

static BLEServer         *pServer = NULL;
static BLECharacteristic *pTxCharacteristic;
static bool               deviceConnected = false;

static uint8_t bsp_ble_rx_buffer[BSP_BLE_RX_BUFFER_SIZE];
static size_t  bsp_ble_rx_len = 0;

/* Private function prototypes ---------------------------------------- */
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    LOG_DBG("BLE connected");
    if (ble_callback)
    {
      ble_callback(BSP_BLE_EVT_CONNECT);
    }
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    LOG_DBG("BLE disconnected");
    if (ble_callback)
    {
      ble_callback(BSP_BLE_EVT_DISCONNECT);
    }
  }
};

class ble_callbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {
      // Copy to static rx buffer safely
      bsp_ble_rx_len = rxValue.length();
      if (bsp_ble_rx_len > BSP_BLE_RX_BUFFER_SIZE)
      {
        bsp_ble_rx_len = BSP_BLE_RX_BUFFER_SIZE;
      }

      memcpy(bsp_ble_rx_buffer, rxValue.c_str(), bsp_ble_rx_len);

      LOG_DBG("BLE Received: %d bytes", bsp_ble_rx_len);

      if (ble_callback)
      {
        ble_callback(BSP_BLE_EVT_RECEIVE_DATA);
      }
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

  // Create the BLE Service
  BLEService *pService = pServer->createService(Nordic_UART_SERVICE_UUID);

  // Create a BLE Characteristic for TX
  pTxCharacteristic =
    pService->createCharacteristic(Nordic_UART_CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE Characteristic for RX
  BLECharacteristic *pRxCharacteristic =
    pService->createCharacteristic(Nordic_UART_CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new ble_callbacks());

  // Start the service
  pService->start();

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

  // Clear after get? Depend on requirements but usually yes
  bsp_ble_rx_len = 0;

  return copy_len;
}

/* End of file -------------------------------------------------------- */
