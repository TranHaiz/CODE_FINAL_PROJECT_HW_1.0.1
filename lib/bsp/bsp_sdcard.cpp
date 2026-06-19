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

#include "bsp_error.h"
#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"

#include <Arduino.h>

LOG_MODULE_REGISTER(bsp_sdcard, LOG_LEVEL_BSP_SDCARD);

/* Private defines ---------------------------------------------------- */
#define SDCARD_MOUNT_RETRIES    (3)
#define SDCARD_MUTEX_TIMEOUT_MS (1000)  // bounded wait so a stuck transfer never hangs a caller

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static SPIClass *sdcard_spi_handler    = nullptr;
static bool      is_sdcard_initialized = false;
static bool      is_sdcard_mounted     = false;
OS_MUTEX_DEFINE_STATIC(sdcard);

/* Private function prototypes ---------------------------------------- */
static bool sdcard_lock(void);
static void sdcard_unlock(void);

/* Function definitions ----------------------------------------------- */
static bool sdcard_lock(void)
{
  if (sdcard_mutex == NULL)
  {
    return true;  // created in bsp_sdcard_init(); boot phase is single-threaded
  }
  return OS_MUTEX_ACQUIRE(sdcard_mutex, SDCARD_MUTEX_TIMEOUT_MS) == pdTRUE;
}

static void sdcard_unlock(void)
{
  if (sdcard_mutex != NULL)
  {
    OS_MUTEX_RELEASE(sdcard_mutex);
  }
}

status_function_t bsp_sdcard_init(void)
{
  OS_MUTEX_CREATE(sdcard);
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
    for (uint8_t file_handle = 0; file_handle < 3; file_handle++)
    {
      LOG_DBG("Trying %lu Hz (attempt %d)...\n", freqs[file_handle], retry + 1);

      if (SD.begin(BSP_SDCARD_DEFAULT_CS_PIN, *sdcard_spi_handler, freqs[file_handle]))
      {
        LOG_INF("Mounted at %lu Hz\n", freqs[file_handle]);
        is_sdcard_mounted = true;
        return STATUS_OK;
      }

      SD.end();
      delay(200);
    }
  }

  bsp_error_handler(BSP_ERROR_SD_INIT);
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

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (open %s)", path);
    return STATUS_BUSY;
  }

  // Select Arduino file mode
  const char *file_mode;
  switch (mode)
  {
  case BSP_SDCARD_MODE_WRITE:
  {
    file_mode = "w";
    break;
  }
  case BSP_SDCARD_MODE_APPEND:
  {
    file_mode = FILE_APPEND;
    break;
  }

  case BSP_SDCARD_MODE_READ:
  default:
  {
    if (!SD.exists(path))
    {
      file->is_open = false;
      sdcard_unlock();
      return STATUS_ERROR;
    }
    file_mode = FILE_READ;
    break;
  }
  }

  file->file = SD.open(path, file_mode);
  if (!file->file)
  {
    LOG_ERR("Cannot open: %s", path);
    file->is_open = false;
    sdcard_unlock();
    return STATUS_ERROR;
  }

  file->is_open  = true;
  file->position = (mode == BSP_SDCARD_MODE_APPEND) ? file->file.size() : 0;
  sdcard_unlock();
  return STATUS_OK;
}

status_function_t bsp_sdcard_seek(bsp_sdcard_file_t *file, size_t position)
{
  if (file == nullptr || !file->is_open)
  {
    return STATUS_ERROR;
  }

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (seek)");
    return STATUS_BUSY;
  }

  bool ok = file->file.seek(position);
  sdcard_unlock();

  if (!ok)
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

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (write)");
    return STATUS_BUSY;
  }

  size_t written = file->file.write(data, len);
  file->file.flush();
  sdcard_unlock();

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

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (read)");
    return STATUS_BUSY;
  }

  *read_len = file->file.read(data, max_len);
  sdcard_unlock();

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
    if (!sdcard_lock())
    {
      LOG_WRN("SD busy (close)");
      return STATUS_BUSY;
    }
    file->file.close();
    sdcard_unlock();

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

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (delete %s)", path);
    return STATUS_BUSY;
  }

  bool ok = (SD.remove(path) || SD.rmdir(path));
  sdcard_unlock();

  if (ok)
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

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (rename %s)", old_path);
    return STATUS_BUSY;
  }

  bool ok = SD.rename(old_path, new_path);
  sdcard_unlock();

  if (ok)
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

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (mkdir %s)", path);
    return STATUS_BUSY;
  }

  bool ok = SD.mkdir(path);
  sdcard_unlock();

  if (ok)
  {
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to create directory: %s\n", path);
  return STATUS_ERROR;
}

status_function_t bsp_sdcard_file_exists(const char *path)
{
  if (path == nullptr || !is_sdcard_mounted)
  {
    return STATUS_ERROR;
  }

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (file_exists %s)", path);
    return STATUS_BUSY;
  }

  bool exists = false;
  if (SD.exists(path))
  {
    File f = SD.open(path, FILE_READ);
    exists = !f.isDirectory();
    f.close();
  }
  sdcard_unlock();

  return exists ? STATUS_OK : STATUS_ERROR;
}

status_function_t bsp_sdcard_dir_exists(const char *path)
{
  if (path == nullptr || !is_sdcard_mounted)
  {
    return STATUS_ERROR;
  }

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (dir_exists %s)", path);
    return STATUS_BUSY;
  }

  File file_handle = SD.open(path, FILE_READ);
  bool is_dir      = (file_handle && file_handle.isDirectory());
  file_handle.close();
  sdcard_unlock();

  return is_dir ? STATUS_OK : STATUS_ERROR;
}

status_function_t bsp_sdcard_dir_open(const char *path, bsp_sdcard_dir_t *dir)
{
  if (path == nullptr || dir == nullptr || !is_sdcard_mounted)
    return STATUS_ERROR;

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (dir_open %s)", path);
    return STATUS_BUSY;
  }

  dir->dir    = SD.open(path, FILE_READ);
  bool is_dir = (dir->dir && dir->dir.isDirectory());
  if (!is_dir)
  {
    dir->dir.close();
  }
  sdcard_unlock();

  return is_dir ? STATUS_OK : STATUS_ERROR;
}

status_function_t bsp_sdcard_dir_read_next(bsp_sdcard_dir_t *dir, char *file_name, size_t max_len)
{
  if (dir == nullptr || file_name == nullptr || max_len == 0)
    return STATUS_ERROR;

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (dir_read_next)");
    return STATUS_BUSY;
  }

  File entry = dir->dir.openNextFile();
  if (!entry)
  {
    sdcard_unlock();
    return STATUS_ERROR;
  }

  const char *full_name = entry.name();
  const char *basename  = strrchr(full_name, '/');
  basename              = (basename != nullptr) ? basename + 1 : full_name;

  strncpy(file_name, basename, max_len - 1);
  file_name[max_len - 1] = '\0';
  entry.close();
  sdcard_unlock();

  return STATUS_OK;
}

status_function_t bsp_sdcard_dir_close(bsp_sdcard_dir_t *dir)
{
  if (dir == nullptr)
    return STATUS_ERROR;

  if (!sdcard_lock())
  {
    LOG_WRN("SD busy (dir_close)");
    return STATUS_BUSY;
  }
  dir->dir.close();
  sdcard_unlock();

  return STATUS_OK;
}

status_function_t bsp_sdcard_deinit(void)
{
  if (is_sdcard_mounted)
  {
    sdcard_lock();  // wait out any in-flight transaction before tearing down the bus
    SD.end();
    is_sdcard_mounted = false;
    sdcard_unlock();
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
