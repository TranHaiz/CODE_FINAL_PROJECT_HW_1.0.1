/**
 * @file       bsp_usb.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-15
 * @author     Hai Tran
 *
 * @brief      BSP USB CDC driver definitions
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_usb.h"

#include "log_service.h"

#include <Arduino.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_usb, LOG_LEVEL_INFO);

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  bsp_usb_callback_t callback;
  bool               is_initialized;
  bool               is_connected;
} bsp_usb_ctx_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static bsp_usb_ctx_t usb_ctx = {
  .callback       = nullptr,
  .is_initialized = false,
  .is_connected   = false,
};

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_usb_init(bsp_usb_callback_t callback)
{
  if (usb_ctx.is_initialized)
  {
    return;
  }

  if (callback == nullptr)
  {
    LOG_WRN("USB init: no callback provided");
  }

  usb_ctx.callback       = callback;
  usb_ctx.is_initialized = true;
  usb_ctx.is_connected   = (bool) Serial;  // true if host already connected

  LOG_DBG("USB CDC initialized");
  return;
}

void bsp_usb_process(void)
{
  if (!usb_ctx.is_initialized)
  {
    return;
  }

  bool connected_now = (bool) Serial;

  // Detect connect
  if (connected_now && !usb_ctx.is_connected)
  {
    usb_ctx.is_connected = true;
    if (usb_ctx.callback)
      usb_ctx.callback(BSP_USB_EVENT_CONNECTED, nullptr);
    LOG_DBG("CDC connected");
  }

  // Detect disconnect
  if (!connected_now && usb_ctx.is_connected)
  {
    usb_ctx.is_connected = false;
    if (usb_ctx.callback)
      usb_ctx.callback(BSP_USB_EVENT_DISCONNECTED, nullptr);
    LOG_DBG("CDC disconnected");
  }

  // Detect incoming data
  if (connected_now && Serial.available() > 0)
  {
    if (usb_ctx.callback)
      usb_ctx.callback(BSP_USB_EVENT_DATA_RX, nullptr);
    LOG_DBG("DATA from USB");
  }
}

status_function_t bsp_usb_deinit(void)
{
  if (!usb_ctx.is_initialized)
  {
    return STATUS_OK;
  }

  usb_ctx.is_initialized = false;
  usb_ctx.is_connected   = false;

  LOG_DBG("USB deinitialized");
  return STATUS_OK;
}

status_function_t bsp_usb_send(const uint8_t *data, size_t len)
{
  if (!usb_ctx.is_initialized)
  {
    return STATUS_ERROR;
  }

  if (data == nullptr || len == 0)
  {
    return STATUS_ERROR;
  }

  if (!usb_ctx.is_connected)
  {
    LOG_WRN("USB send: CDC not connected");
    return STATUS_ERROR;
  }

  size_t written = Serial.write(data, len);
  if (written != len)
  {
    LOG_WRN("USB send: incomplete (%u / %u bytes)", (unsigned) written, (unsigned) len);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

size_t bsp_usb_available(void)
{
  if (!usb_ctx.is_initialized || !usb_ctx.is_connected)
  {
    return 0;
  }

  return (size_t) Serial.available();
}

size_t bsp_usb_read(uint8_t *buf, size_t max_len)
{
  if (!usb_ctx.is_initialized || !usb_ctx.is_connected)
  {
    return 0;
  }

  if (buf == nullptr || max_len == 0)
  {
    return 0;
  }

  return (size_t) Serial.readBytes(buf, max_len);
}

/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */