/**
 * @file       bsp_sdcard.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.1.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      Simplified BSP implementation for SD Card (single instance)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_sdcard.h"

#include "device_config.h"
#include "log_service.h"

#include <Arduino.h>

LOG_MODULE_REGISTER(bsp_sdcard, LOG_LEVEL_DBG);

/* Private defines ---------------------------------------------------- */
#define SDCARD_MOUNT_RETRIES (3)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static SPIClass *sdcard_spi_handler    = nullptr;
static bool      is_sdcard_initialized = false;
static bool      is_sdcard_mounted     = false;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

status_function_t bsp_sdcard_init(void)
{
  sdcard_spi_handler = new SPIClass(FSPI);
  if (sdcard_spi_handler == nullptr)
  {
    LOG_ERR("Failed to create SPI instance");
    return STATUS_ERROR;
  }

  if (!is_sdcard_initialized)
  {
    is_sdcard_initialized = true;
  }

  // Configure CS pin
  pinMode(BSP_SDCARD_DEFAULT_CS_PIN, OUTPUT);
  digitalWrite(BSP_SDCARD_DEFAULT_CS_PIN, HIGH);
  delay(100);

  // Initialize SPI
  sdcard_spi_handler->begin(BSP_SDCARD_DEFAULT_SCK_PIN, BSP_SDCARD_DEFAULT_MISO_PIN, BSP_SDCARD_DEFAULT_MOSI_PIN,
                            BSP_SDCARD_DEFAULT_CS_PIN);
  delay(100);

  is_sdcard_initialized = true;

  // Try mounting with different frequencies
  uint32_t freqs[] = { SDCARD_MAX_SPI_FREQ, SDCARD_NORMAL_SPI_FREQ, SDCARD_MIN_SPI_FREQ };

  for (uint8_t retry = 0; retry < SDCARD_MOUNT_RETRIES; retry++)
  {
    for (uint8_t f = 0; f < 3; f++)
    {
      LOG_DBG("Trying %lu Hz (attempt %d)...\n", freqs[f], retry + 1);

      if (SD.begin(BSP_SDCARD_DEFAULT_CS_PIN, *sdcard_spi_handler, freqs[f]))
      {
        LOG_INF("Mounted at %lu Hz\n", freqs[f]);
        is_sdcard_mounted = true;
        return STATUS_OK;
      }

      SD.end();
      delay(200);
    }
  }

  return STATUS_ERROR;
}

status_function_t bsp_sdcard_is_mounted(void)
{
  return is_sdcard_mounted ? STATUS_OK : STATUS_ERROR;
}

status_function_t bsp_sdcard_open(const char *path, bsp_sdcard_mode_t mode, bsp_sdcard_file_t *file)
{
  if ((path == nullptr) || (file == nullptr) || (!is_sdcard_mounted))
  {
    return STATUS_ERROR;
  }

  // Select Arduino file mode
  const char *file_mode;
  switch (mode)
  {
  case BSP_SDCARD_MODE_WRITE: file_mode = FILE_WRITE; break;
  case BSP_SDCARD_MODE_APPEND: file_mode = FILE_APPEND; break;
  case BSP_SDCARD_MODE_READ:
  default: file_mode = FILE_READ; break;
  }

  file->file = SD.open(path, file_mode);
  if (!file->file)
  {
    LOG_ERR("Cannot open: %s", path);
    file->is_open = false;
    return STATUS_ERROR;
  }

  file->is_open  = true;
  file->position = (mode == BSP_SDCARD_MODE_APPEND) ? file->file.size() : 0;
  return STATUS_OK;
}

status_function_t bsp_sdcard_seek(bsp_sdcard_file_t *file, size_t position)
{
  if (file == nullptr || !file->is_open)
  {
    return STATUS_ERROR;
  }

  if (!file->file.seek(position))
  {
    return STATUS_ERROR;
  }

  file->position = position;
  return STATUS_OK;
}

status_function_t bsp_sdcard_write(bsp_sdcard_file_t *file, const uint8_t *data, size_t len, size_t *written_len)
{
  if (file == nullptr || data == nullptr || !file->is_open)
  {
    return STATUS_ERROR;
  }

  size_t written = file->file.write(data, len);
  file->file.flush();
  file->position += written;

  if (written_len != nullptr)
  {
    *written_len = written;
  }

  return (written == len) ? STATUS_OK : STATUS_ERROR;
}

status_function_t bsp_sdcard_read(bsp_sdcard_file_t *file, uint8_t *data, size_t max_len, size_t *read_len)
{
  if (file == nullptr || data == nullptr || read_len == nullptr || !file->is_open)
  {
    return STATUS_ERROR;
  }

  *read_len = file->file.read(data, max_len);
  file->position += *read_len;
  return STATUS_OK;
}

status_function_t bsp_sdcard_close(bsp_sdcard_file_t *file)
{
  if (file == nullptr)
  {
    return STATUS_ERROR;
  }

  if (file->is_open)
  {
    file->file.close();
    file->is_open  = false;
    file->position = 0;
  }

  return STATUS_OK;
}

status_function_t bsp_sdcard_delete(const char *path)
{
  if (path == nullptr || !is_sdcard_mounted)
  {
    return STATUS_ERROR;
  }

  if (SD.remove(path) || SD.rmdir(path))
  {
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to delete: %s\n", path);
  return STATUS_ERROR;
}

status_function_t bsp_sdcard_rename(const char *old_path, const char *new_path)
{
  if (old_path == nullptr || new_path == nullptr || !is_sdcard_mounted)
  {
    return STATUS_ERROR;
  }

  if (SD.rename(old_path, new_path))
  {
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to rename: %s -> %s\n", old_path, new_path);
  return STATUS_ERROR;
}

status_function_t bsp_sdcard_mkdir(const char *path)
{
  if (path == nullptr || !is_sdcard_mounted)
  {
    return STATUS_ERROR;
  }

  if (SD.mkdir(path))
  {
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to create directory: %s\n", path);
  return STATUS_ERROR;
}

status_function_t bsp_sdcard_deinit(void)
{
  if (is_sdcard_mounted)
  {
    SD.end();
    is_sdcard_mounted = false;
  }

  if (sdcard_spi_handler != nullptr)
  {
    sdcard_spi_handler->end();
    delete sdcard_spi_handler;
    sdcard_spi_handler = nullptr;
  }

  is_sdcard_initialized = false;
  Serial.println("[SDCARD] Deinitialized");
  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
