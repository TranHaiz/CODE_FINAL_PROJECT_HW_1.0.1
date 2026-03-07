/**
 * @file       bsp_sdcard.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP implementation for SD Card using SPI interface
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_sdcard.h"

#include "device_pin_conf.h"
#include "os_lib.h"

#include <Arduino.h>
#include <stdarg.h>

/* Private defines ---------------------------------------------------- */
#define SDCARD_TEST_FILE   "/test_rw.bin"
#define SDCARD_RESULT_FILE "/test_result.txt"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static const char *s_card_type_strings[] = { "NONE", "MMC", "SD", "SDHC/SDXC", "UNKNOWN" };

/* Private function prototypes ---------------------------------------- */
static status_function_t bsp_sdcard_update_info(bsp_sdcard_t *ctx);

/* Function definitions ----------------------------------------------- */

status_function_t bsp_sdcard_init(bsp_sdcard_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  // Set default configuration
  bsp_sdcard_config_t default_config = {
    .cs_pin        = BSP_SDCARD_DEFAULT_CS_PIN,
    .sck_pin       = BSP_SDCARD_DEFAULT_SCK_PIN,
    .miso_pin      = BSP_SDCARD_DEFAULT_MISO_PIN,
    .mosi_pin      = BSP_SDCARD_DEFAULT_MOSI_PIN,
    .spi_freq      = BSP_SDCARD_DEFAULT_SPI_FREQ,
    .mount_retries = BSP_SDCARD_DEFAULT_MOUNT_RETRIES,
  };

  return bsp_sdcard_init_with_config(ctx, &default_config);
}

status_function_t bsp_sdcard_init_with_config(bsp_sdcard_t *ctx, const bsp_sdcard_config_t *config)
{
  if (ctx == nullptr || config == nullptr)
  {
    return STATUS_ERROR;
  }

  // Copy configuration
  ctx->config = *config;

  // Initialize info structure
  ctx->info.card_type       = BSP_SDCARD_TYPE_NONE;
  ctx->info.card_size_bytes = 0;
  ctx->info.total_bytes     = 0;
  ctx->info.used_bytes      = 0;
  ctx->info.free_bytes      = 0;
  ctx->info.sector_size     = BSP_SDCARD_DEFAULT_BLOCK_SIZE;
  ctx->info.is_mounted      = false;

  // Create SPI instance (FSPI/SPI2 - separate from HSPI used by ILI9341 display)
  ctx->spi = new SPIClass(FSPI);

  if (ctx->spi == nullptr)
  {
    Serial.println("[SDCARD] ERROR: Failed to create SPI instance");
    return STATUS_ERROR;
  }

  // Configure CS pin
  pinMode(config->cs_pin, OUTPUT);
  digitalWrite(config->cs_pin, HIGH);
  delay(100);

  // Initialize SPI
  ctx->spi->begin(config->sck_pin, config->miso_pin, config->mosi_pin, config->cs_pin);
  delay(100);

  ctx->is_initialized = true;

  Serial.printf("[SDCARD] Initialized - CS=%d, SCK=%d, MISO=%d, MOSI=%d, Freq=%luHz\n", config->cs_pin, config->sck_pin,
                config->miso_pin, config->mosi_pin, config->spi_freq);

  return STATUS_OK;
}

status_function_t bsp_sdcard_mount(bsp_sdcard_t *ctx)
{
  if (ctx == nullptr || !ctx->is_initialized)
  {
    return STATUS_ERROR;
  }

  if (ctx->info.is_mounted)
  {
    Serial.println("[SDCARD] Already mounted");
    return STATUS_OK;
  }

  // Try mounting with different frequencies
  uint32_t freqs[]  = { ctx->config.spi_freq, 4000000, 1000000 };
  bool     mounted  = false;
  uint8_t  num_freq = sizeof(freqs) / sizeof(freqs[0]);

  for (uint8_t retry = 0; retry < ctx->config.mount_retries; retry++)
  {
    for (uint8_t f = 0; f < num_freq; f++)
    {
      Serial.printf("[SDCARD] Trying frequency: %lu Hz (attempt %d)...\n", freqs[f], retry + 1);

      if (SD.begin(ctx->config.cs_pin, *ctx->spi, freqs[f]))
      {
        Serial.printf("[SDCARD] Mounted successfully at %lu Hz\n", freqs[f]);
        ctx->config.spi_freq = freqs[f];
        mounted              = true;
        break;
      }

      SD.end();
      delay(200);
    }

    if (mounted)
      break;
  }

  if (!mounted)
  {
    Serial.println("[SDCARD] ERROR: Failed to mount SD card");
    return STATUS_ERROR;
  }

  ctx->info.is_mounted = true;

  // Update card information
  bsp_sdcard_update_info(ctx);

  // Print card info
  Serial.printf("[SDCARD] Card Type: %s\n", bsp_sdcard_type_to_str(ctx->info.card_type));
  Serial.printf("[SDCARD] Card Size: %llu MB\n", ctx->info.card_size_bytes / (1024 * 1024));
  Serial.printf("[SDCARD] Total: %llu MB | Used: %llu MB | Free: %llu MB\n", ctx->info.total_bytes / (1024 * 1024),
                ctx->info.used_bytes / (1024 * 1024), ctx->info.free_bytes / (1024 * 1024));

  return STATUS_OK;
}

status_function_t bsp_sdcard_unmount(bsp_sdcard_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  SD.end();
  ctx->info.is_mounted = false;

  Serial.println("[SDCARD] Unmounted");
  return STATUS_OK;
}

status_function_t bsp_sdcard_get_info(bsp_sdcard_t *ctx, bsp_sdcard_info_t *info)
{
  if (ctx == nullptr || info == nullptr)
  {
    return STATUS_ERROR;
  }

  if (ctx->info.is_mounted)
  {
    bsp_sdcard_update_info(ctx);
  }

  *info = ctx->info;
  return STATUS_OK;
}

const char *bsp_sdcard_type_to_str(bsp_sdcard_type_t type)
{
  if (type < 5)
  {
    return s_card_type_strings[type];
  }
  return "?";
}

bool bsp_sdcard_file_exists(bsp_sdcard_t *ctx, const char *path)
{
  if (ctx == nullptr || path == nullptr || !ctx->info.is_mounted)
  {
    return false;
  }

  return SD.exists(path);
}

status_function_t bsp_sdcard_mkdir(bsp_sdcard_t *ctx, const char *path)
{
  if (ctx == nullptr || path == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  if (SD.mkdir(path))
  {
    Serial.printf("[SDCARD] Created directory: %s\n", path);
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to create directory: %s\n", path);
  return STATUS_ERROR;
}

status_function_t bsp_sdcard_remove(bsp_sdcard_t *ctx, const char *path)
{
  if (ctx == nullptr || path == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  if (SD.remove(path))
  {
    return STATUS_OK;
  }

  // Try removing as directory
  if (SD.rmdir(path))
  {
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to remove: %s\n", path);
  return STATUS_ERROR;
}

status_function_t bsp_sdcard_rename(bsp_sdcard_t *ctx, const char *old_path, const char *new_path)
{
  if (ctx == nullptr || old_path == nullptr || new_path == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  if (SD.rename(old_path, new_path))
  {
    Serial.printf("[SDCARD] Renamed: %s -> %s\n", old_path, new_path);
    return STATUS_OK;
  }

  Serial.printf("[SDCARD] ERROR: Failed to rename: %s\n", old_path);
  return STATUS_ERROR;
}

status_function_t
bsp_sdcard_write_file(bsp_sdcard_t *ctx, const char *path, const uint8_t *data, size_t len, bool append)
{
  if (ctx == nullptr || path == nullptr || data == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  File file = SD.open(path, append ? FILE_APPEND : FILE_WRITE);
  if (!file)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open file for writing: %s\n", path);
    return STATUS_ERROR;
  }

  size_t written = file.write(data, len);
  file.close();

  if (written != len)
  {
    Serial.printf("[SDCARD] ERROR: Write incomplete - %u/%u bytes\n", written, len);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_sdcard_write_string(bsp_sdcard_t *ctx, const char *path, const char *str, bool append)
{
  if (str == nullptr)
  {
    return STATUS_ERROR;
  }

  return bsp_sdcard_write_file(ctx, path, (const uint8_t *) str, strlen(str), append);
}

status_function_t
bsp_sdcard_read_file(bsp_sdcard_t *ctx, const char *path, uint8_t *data, size_t max_len, size_t *read_len)
{
  if (ctx == nullptr || path == nullptr || data == nullptr || read_len == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  File file = SD.open(path, FILE_READ);
  if (!file)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open file for reading: %s\n", path);
    return STATUS_ERROR;
  }

  *read_len = file.read(data, max_len);
  file.close();

  return STATUS_OK;
}

status_function_t bsp_sdcard_get_file_size(bsp_sdcard_t *ctx, const char *path, size_t *size)
{
  if (ctx == nullptr || path == nullptr || size == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  File file = SD.open(path, FILE_READ);
  if (!file)
  {
    return STATUS_ERROR;
  }

  *size = file.size();
  file.close();

  return STATUS_OK;
}

status_function_t bsp_sdcard_list_dir(bsp_sdcard_t *ctx, const char *path)
{
  if (ctx == nullptr || path == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  File root = SD.open(path);
  if (!root)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open directory: %s\n", path);
    return STATUS_ERROR;
  }

  if (!root.isDirectory())
  {
    Serial.printf("[SDCARD] ERROR: Not a directory: %s\n", path);
    root.close();
    return STATUS_ERROR;
  }

  Serial.printf("[SDCARD] Directory listing: %s\n", path);
  Serial.println("--------------------------------------------");

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.printf("  [DIR]  %s\n", file.name());
    }
    else
    {
      Serial.printf("  [FILE] %s (%lu bytes)\n", file.name(), file.size());
    }
    file = root.openNextFile();
  }

  Serial.println("--------------------------------------------");
  root.close();

  return STATUS_OK;
}

status_function_t bsp_sdcard_benchmark(bsp_sdcard_t                   *ctx,
                                       uint16_t                        rounds,
                                       uint16_t                        block_size,
                                       uint16_t                        block_count,
                                       bsp_sdcard_benchmark_summary_t *summary)
{
  if (ctx == nullptr || summary == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  if (rounds == 0 || block_size == 0 || block_count == 0)
  {
    return STATUS_ERROR;
  }

  // Allocate buffers
  uint8_t *write_buf = (uint8_t *) malloc(block_size);
  uint8_t *read_buf  = (uint8_t *) malloc(block_size);

  if (write_buf == nullptr || read_buf == nullptr)
  {
    Serial.println("[SDCARD] ERROR: Failed to allocate benchmark buffers");
    if (write_buf)
      free(write_buf);
    if (read_buf)
      free(read_buf);
    return STATUS_ERROR;
  }

  // Fill write buffer with pattern
  for (uint16_t i = 0; i < block_size; i++)
  {
    write_buf[i] = (uint8_t) (i & 0xFF);
  }

  // Initialize summary
  summary->max_write_kbs      = 0;
  summary->min_write_kbs      = 1e9f;
  summary->avg_write_kbs      = 0;
  summary->max_read_kbs       = 0;
  summary->min_read_kbs       = 1e9f;
  summary->avg_read_kbs       = 0;
  summary->max_verify_kbs     = 0;
  summary->min_verify_kbs     = 1e9f;
  summary->avg_verify_kbs     = 0;
  summary->total_errors       = 0;
  summary->total_bytes_tested = 0;
  summary->rounds_completed   = 0;

  uint32_t bytes_per_round = (uint32_t) block_size * block_count;

  Serial.println("\n==============================");
  Serial.printf("SD CARD BENCHMARK: %d rounds | %lu KB/round\n", rounds, bytes_per_round / 1024);
  Serial.println("==============================");

  for (uint16_t round = 0; round < rounds; round++)
  {
    Serial.printf("\n--- Round %d/%d ---\n", round + 1, rounds);

    // ── WRITE ──
    File f = SD.open(SDCARD_TEST_FILE, FILE_WRITE);
    if (!f)
    {
      Serial.println("[SDCARD] ERROR: Cannot open file for write");
      continue;
    }

    uint32_t t0 = micros();
    for (uint16_t b = 0; b < block_count; b++)
    {
      f.write(write_buf, block_size);
    }
    f.flush();
    f.close();

    float write_time = (micros() - t0) / 1e6f;
    float write_kbs  = (bytes_per_round / 1024.0f) / write_time;

    // ── READ ──
    f = SD.open(SDCARD_TEST_FILE, FILE_READ);
    if (!f)
    {
      Serial.println("[SDCARD] ERROR: Cannot open file for read");
      continue;
    }

    t0 = micros();
    for (uint16_t b = 0; b < block_count; b++)
    {
      f.read(read_buf, block_size);
    }
    f.close();

    float read_time = (micros() - t0) / 1e6f;
    float read_kbs  = (bytes_per_round / 1024.0f) / read_time;

    // ── VERIFY ──
    uint32_t error_count = 0;

    f = SD.open(SDCARD_TEST_FILE, FILE_READ);
    if (!f)
    {
      Serial.println("[SDCARD] ERROR: Cannot open file for verify");
      continue;
    }

    t0 = micros();
    for (uint16_t b = 0; b < block_count; b++)
    {
      int bytes_read = f.read(read_buf, block_size);
      for (int i = 0; i < bytes_read; i++)
      {
        uint8_t expected = (uint8_t) (i & 0xFF);
        if (read_buf[i] != expected)
        {
          error_count++;
        }
      }
    }
    f.close();

    float verify_time = (micros() - t0) / 1e6f;
    float verify_kbs  = (bytes_per_round / 1024.0f) / verify_time;

    // Print round result
    Serial.printf("  Write:  %.1f KB/s\n", write_kbs);
    Serial.printf("  Read:   %.1f KB/s\n", read_kbs);
    Serial.printf("  Verify: %.1f KB/s | %s\n", verify_kbs, error_count == 0 ? "OK" : "ERRORS!");

    // Update summary
    summary->max_write_kbs = max(summary->max_write_kbs, write_kbs);
    summary->min_write_kbs = min(summary->min_write_kbs, write_kbs);
    summary->avg_write_kbs += write_kbs;

    summary->max_read_kbs = max(summary->max_read_kbs, read_kbs);
    summary->min_read_kbs = min(summary->min_read_kbs, read_kbs);
    summary->avg_read_kbs += read_kbs;

    summary->max_verify_kbs = max(summary->max_verify_kbs, verify_kbs);
    summary->min_verify_kbs = min(summary->min_verify_kbs, verify_kbs);
    summary->avg_verify_kbs += verify_kbs;

    summary->total_errors += error_count;
    summary->total_bytes_tested += bytes_per_round;
    summary->rounds_completed++;
  }

  // Calculate averages
  if (summary->rounds_completed > 0)
  {
    summary->avg_write_kbs /= summary->rounds_completed;
    summary->avg_read_kbs /= summary->rounds_completed;
    summary->avg_verify_kbs /= summary->rounds_completed;
  }

  // Cleanup
  SD.remove(SDCARD_TEST_FILE);
  free(write_buf);
  free(read_buf);

  // Print summary
  Serial.println("\n==============================");
  Serial.println("BENCHMARK SUMMARY");
  Serial.println("==============================");
  Serial.printf("Write: Max=%.1f | Min=%.1f | Avg=%.1f KB/s\n", summary->max_write_kbs, summary->min_write_kbs,
                summary->avg_write_kbs);
  Serial.printf("Read:  Max=%.1f | Min=%.1f | Avg=%.1f KB/s\n", summary->max_read_kbs, summary->min_read_kbs,
                summary->avg_read_kbs);
  Serial.printf("Verify: Max=%.1f | Min=%.1f | Avg=%.1f KB/s\n", summary->max_verify_kbs, summary->min_verify_kbs,
                summary->avg_verify_kbs);
  Serial.printf("Total errors: %lu / %lu bytes\n", summary->total_errors, summary->total_bytes_tested);

  return STATUS_OK;
}

status_function_t bsp_sdcard_quick_test(bsp_sdcard_t *ctx, bsp_sdcard_benchmark_result_t *result)
{
  if (ctx == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  const uint16_t block_size  = 512;
  const uint16_t block_count = 100;  // 50KB test
  uint32_t       total_bytes = (uint32_t) block_size * block_count;

  uint8_t *write_buf = (uint8_t *) malloc(block_size);
  uint8_t *read_buf  = (uint8_t *) malloc(block_size);

  if (write_buf == nullptr || read_buf == nullptr)
  {
    if (write_buf)
      free(write_buf);
    if (read_buf)
      free(read_buf);
    return STATUS_ERROR;
  }

  // Fill pattern
  for (uint16_t i = 0; i < block_size; i++)
  {
    write_buf[i] = (uint8_t) (i & 0xFF);
  }

  Serial.println("[SDCARD] Running quick test...");

  // Write
  File f = SD.open("/quick_test.bin", FILE_WRITE);
  if (!f)
  {
    free(write_buf);
    free(read_buf);
    return STATUS_ERROR;
  }

  uint32_t t0 = micros();
  for (uint16_t b = 0; b < block_count; b++)
  {
    f.write(write_buf, block_size);
  }
  f.flush();
  f.close();

  result->time_write_s    = (micros() - t0) / 1e6f;
  result->write_speed_kbs = (total_bytes / 1024.0f) / result->time_write_s;

  // Read
  f = SD.open("/quick_test.bin", FILE_READ);
  if (!f)
  {
    free(write_buf);
    free(read_buf);
    return STATUS_ERROR;
  }

  t0 = micros();
  for (uint16_t b = 0; b < block_count; b++)
  {
    f.read(read_buf, block_size);
  }
  f.close();

  result->time_read_s    = (micros() - t0) / 1e6f;
  result->read_speed_kbs = (total_bytes / 1024.0f) / result->time_read_s;

  // Verify
  result->error_count = 0;
  f                   = SD.open("/quick_test.bin", FILE_READ);
  if (!f)
  {
    free(write_buf);
    free(read_buf);
    return STATUS_ERROR;
  }

  t0 = micros();
  for (uint16_t b = 0; b < block_count; b++)
  {
    int bytes_read = f.read(read_buf, block_size);
    for (int i = 0; i < bytes_read; i++)
    {
      if (read_buf[i] != (uint8_t) (i & 0xFF))
      {
        result->error_count++;
      }
    }
  }
  f.close();

  result->time_verify_s    = (micros() - t0) / 1e6f;
  result->verify_speed_kbs = (total_bytes / 1024.0f) / result->time_verify_s;

  // Cleanup
  SD.remove("/quick_test.bin");
  free(write_buf);
  free(read_buf);

  Serial.printf("[SDCARD] Quick test: W=%.1f R=%.1f V=%.1f KB/s | Errors=%lu\n", result->write_speed_kbs,
                result->read_speed_kbs, result->verify_speed_kbs, result->error_count);

  if (result != nullptr)
  {
    return (result->error_count == 0) ? STATUS_OK : STATUS_ERROR;
  }

  return STATUS_OK;
}

bool bsp_sdcard_is_mounted(bsp_sdcard_t *ctx)
{
  if (ctx == nullptr)
  {
    return false;
  }

  return ctx->info.is_mounted;
}

status_function_t bsp_sdcard_deinit(bsp_sdcard_t *ctx)
{
  if (ctx == nullptr)
  {
    return STATUS_ERROR;
  }

  if (ctx->info.is_mounted)
  {
    bsp_sdcard_unmount(ctx);
  }

  if (ctx->spi != nullptr)
  {
    ctx->spi->end();
    delete ctx->spi;
    ctx->spi = nullptr;
  }

  ctx->is_initialized = false;

  Serial.println("[SDCARD] Deinitialized");
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
   CSV Functions
   ═══════════════════════════════════════════════════════════════════════ */

status_function_t bsp_sdcard_csv_create(bsp_sdcard_t *ctx, const char *path, const char **headers, uint8_t header_count)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_WRITE);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot create CSV file: %s\n", path);
    return STATUS_ERROR;
  }

  // Write headers if provided
  if (headers != nullptr && header_count > 0)
  {
    for (uint8_t i = 0; i < header_count; i++)
    {
      if (i > 0)
      {
        f.print(",");
      }
      f.print(headers[i]);
    }
    f.println();
  }

  f.flush();
  f.close();
  Serial.printf("[SDCARD] CSV created: %s\n", path);
  return STATUS_OK;
}

status_function_t
bsp_sdcard_csv_append_row(bsp_sdcard_t *ctx, const char *path, const char **values, uint8_t value_count)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || values == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open CSV file: %s\n", path);
    return STATUS_ERROR;
  }

  for (uint8_t i = 0; i < value_count; i++)
  {
    if (i > 0)
    {
      f.print(",");
    }
    f.print(values[i]);
  }
  f.println();

  f.flush();
  f.close();
  return STATUS_OK;
}

status_function_t bsp_sdcard_csv_append_floats(bsp_sdcard_t *ctx,
                                               const char   *path,
                                               const float  *values,
                                               uint8_t       value_count,
                                               uint8_t       precision)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || values == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open CSV file: %s\n", path);
    return STATUS_ERROR;
  }

  for (uint8_t i = 0; i < value_count; i++)
  {
    if (i > 0)
    {
      f.print(",");
    }
    f.print(values[i], precision);
  }
  f.println();

  f.flush();
  f.close();
  return STATUS_OK;
}

status_function_t
bsp_sdcard_csv_append_ints(bsp_sdcard_t *ctx, const char *path, const int32_t *values, uint8_t value_count)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || values == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open CSV file: %s\n", path);
    return STATUS_ERROR;
  }

  for (uint8_t i = 0; i < value_count; i++)
  {
    if (i > 0)
    {
      f.print(",");
    }
    f.print(values[i]);
  }
  f.println();

  f.flush();
  f.close();
  return STATUS_OK;
}

status_function_t bsp_sdcard_csv_append_timestamped(bsp_sdcard_t *ctx,
                                                    const char   *path,
                                                    const char   *timestamp,
                                                    const float  *values,
                                                    uint8_t       num_cols,
                                                    uint8_t       precision)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || values == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open CSV file: %s\n", path);
    return STATUS_ERROR;
  }

  // Write timestamp first
  if (timestamp != nullptr)
  {
    f.print(timestamp);
  }

  // Write values
  for (uint8_t i = 0; i < num_cols; i++)
  {
    f.print(",");
    f.print(values[i], precision);
  }
  f.println();

  f.flush();
  f.close();
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
   LOG Functions
   ═══════════════════════════════════════════════════════════════════════ */

const char *bsp_sdcard_log_level_to_str(bsp_sdcard_log_level_t level)
{
  switch (level)
  {
  case BSP_SDCARD_LOG_DEBUG: return "DEBUG";
  case BSP_SDCARD_LOG_INFO: return "INFO";
  case BSP_SDCARD_LOG_WARNING: return "WARN";
  case BSP_SDCARD_LOG_ERROR: return "ERROR";
  case BSP_SDCARD_LOG_CRITICAL: return "CRIT";
  default: return "UNKN";
  }
}

status_function_t bsp_sdcard_log_create(bsp_sdcard_t *ctx, const char *path, const char *header)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_WRITE);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot create LOG file: %s\n", path);
    return STATUS_ERROR;
  }

  // Write header section
  f.println("================================================================================");
  f.printf("LOG FILE: %s\n", path);
  if (header != nullptr)
  {
    f.printf("DESCRIPTION: %s\n", header);
  }
  f.printf("CREATED AT: %lu ms\n", millis());
  f.println("================================================================================");
  f.println();

  f.flush();
  f.close();
  Serial.printf("[SDCARD] LOG created: %s\n", path);
  return STATUS_OK;
}

status_function_t
bsp_sdcard_log_write(bsp_sdcard_t *ctx, const char *path, bsp_sdcard_log_level_t level, const char *message)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || message == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open LOG file: %s\n", path);
    return STATUS_ERROR;
  }

  f.printf("[%s] %s\n", bsp_sdcard_log_level_to_str(level), message);
  f.flush();
  f.close();
  return STATUS_OK;
}

status_function_t bsp_sdcard_log_write_timestamped(bsp_sdcard_t          *ctx,
                                                   const char            *path,
                                                   const char            *timestamp,
                                                   bsp_sdcard_log_level_t level,
                                                   const char            *message)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || message == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open LOG file: %s\n", path);
    return STATUS_ERROR;
  }

  // Format: [TIMESTAMP][LEVEL] message
  if (timestamp != nullptr)
  {
    f.printf("[%s][%s] %s\n", timestamp, bsp_sdcard_log_level_to_str(level), message);
  }
  else
  {
    f.printf("[%s] %s\n", bsp_sdcard_log_level_to_str(level), message);
  }
  f.flush();
  f.close();
  return STATUS_OK;
}

status_function_t
bsp_sdcard_log_printf(bsp_sdcard_t *ctx, const char *path, bsp_sdcard_log_level_t level, const char *format, ...)
{
  if (ctx == nullptr || !ctx->info.is_mounted || path == nullptr || format == nullptr)
  {
    return STATUS_ERROR;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.printf("[SDCARD] ERROR: Cannot open LOG file: %s\n", path);
    return STATUS_ERROR;
  }

  // Format timestamp
  uint32_t timestamp_ms = millis();
  uint32_t hours        = timestamp_ms / 3600000;
  uint32_t mins         = (timestamp_ms % 3600000) / 60000;
  uint32_t secs         = (timestamp_ms % 60000) / 1000;
  uint32_t ms           = timestamp_ms % 1000;

  // Write prefix
  f.printf("[%02lu:%02lu:%02lu.%03lu][%s] ", hours, mins, secs, ms, bsp_sdcard_log_level_to_str(level));

  // Write formatted message
  char    buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  f.println(buffer);
  f.flush();
  f.close();
  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */

static status_function_t bsp_sdcard_update_info(bsp_sdcard_t *ctx)
{
  if (ctx == nullptr || !ctx->info.is_mounted)
  {
    return STATUS_ERROR;
  }

  uint8_t card_type = SD.cardType();
  switch (card_type)
  {
  case CARD_NONE: ctx->info.card_type = BSP_SDCARD_TYPE_NONE; break;
  case CARD_MMC: ctx->info.card_type = BSP_SDCARD_TYPE_MMC; break;
  case CARD_SD: ctx->info.card_type = BSP_SDCARD_TYPE_SD; break;
  case CARD_SDHC: ctx->info.card_type = BSP_SDCARD_TYPE_SDHC; break;
  default: ctx->info.card_type = BSP_SDCARD_TYPE_UNKNOWN; break;
  }

  ctx->info.card_size_bytes = SD.cardSize();
  ctx->info.total_bytes     = SD.totalBytes();
  ctx->info.used_bytes      = SD.usedBytes();
  ctx->info.free_bytes      = ctx->info.total_bytes - ctx->info.used_bytes;

  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
