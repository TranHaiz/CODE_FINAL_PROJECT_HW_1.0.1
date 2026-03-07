/**
 * @file       bsp_sdcard.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      BSP for SD Card using SPI interface
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_SDCARD_H_
#define _BSP_SDCARD_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <FS.h>
#include <SD.h>
#include <SPI.h>

/* Public defines ----------------------------------------------------- */
#define BSP_SDCARD_DEFAULT_CS_PIN        (39)
#define BSP_SDCARD_DEFAULT_SCK_PIN       (38)
#define BSP_SDCARD_DEFAULT_MISO_PIN      (37)
#define BSP_SDCARD_DEFAULT_MOSI_PIN      (36)
#define BSP_SDCARD_DEFAULT_SPI_FREQ      (4000000)   // 4MHz default
#define BSP_SDCARD_MAX_SPI_FREQ          (40000000)  // 40MHz max
#define BSP_SDCARD_DEFAULT_BLOCK_SIZE    (512)
#define BSP_SDCARD_DEFAULT_MOUNT_RETRIES (3)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief SD card type enumeration
 */
typedef enum
{
  BSP_SDCARD_TYPE_NONE    = 0,
  BSP_SDCARD_TYPE_MMC     = 1,
  BSP_SDCARD_TYPE_SD      = 2,
  BSP_SDCARD_TYPE_SDHC    = 3,
  BSP_SDCARD_TYPE_UNKNOWN = 4,
} bsp_sdcard_type_t;

/**
 * @brief SD card information structure
 */
typedef struct
{
  bsp_sdcard_type_t card_type;        // Card type
  uint64_t          card_size_bytes;  // Card size in bytes
  uint64_t          total_bytes;      // Total filesystem bytes
  uint64_t          used_bytes;       // Used bytes
  uint64_t          free_bytes;       // Free bytes
  uint32_t          sector_size;      // Sector size
  bool              is_mounted;       // Mount status
} bsp_sdcard_info_t;

/**
 * @brief SD card benchmark result for one round
 */
typedef struct
{
  float    write_speed_kbs;   // Write speed in KB/s
  float    read_speed_kbs;    // Read speed in KB/s
  float    verify_speed_kbs;  // Verify speed in KB/s
  uint32_t error_count;       // Number of errors
  float    time_write_s;      // Write time in seconds
  float    time_read_s;       // Read time in seconds
  float    time_verify_s;     // Verify time in seconds
} bsp_sdcard_benchmark_result_t;

/**
 * @brief SD card benchmark summary
 */
typedef struct
{
  float    max_write_kbs;
  float    min_write_kbs;
  float    avg_write_kbs;
  float    max_read_kbs;
  float    min_read_kbs;
  float    avg_read_kbs;
  float    max_verify_kbs;
  float    min_verify_kbs;
  float    avg_verify_kbs;
  uint32_t total_errors;
  uint32_t total_bytes_tested;
  uint16_t rounds_completed;
} bsp_sdcard_benchmark_summary_t;

/**
 * @brief SD card configuration structure
 */
typedef struct
{
  uint8_t  cs_pin;         // Chip select pin
  uint8_t  sck_pin;        // Clock pin
  uint8_t  miso_pin;       // MISO pin
  uint8_t  mosi_pin;       // MOSI pin
  uint32_t spi_freq;       // SPI frequency in Hz
  uint8_t  mount_retries;  // Number of mount retries
} bsp_sdcard_config_t;

/**
 * @brief SD card context structure
 */
typedef struct
{
  SPIClass           *spi;             // SPI instance
  bsp_sdcard_config_t config;          // Configuration
  bsp_sdcard_info_t   info;            // Card information
  bool                is_initialized;  // Initialization flag
} bsp_sdcard_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize SD card with default configuration
 *
 * @param[in] ctx     Pointer to SD card context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_init(bsp_sdcard_t *ctx);

/**
 * @brief Initialize SD card with custom configuration
 *
 * @param[in] ctx     Pointer to SD card context
 * @param[in] config  Pointer to configuration structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_init_with_config(bsp_sdcard_t *ctx, const bsp_sdcard_config_t *config);

/**
 * @brief Mount SD card
 *
 * @param[in] ctx     Pointer to SD card context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_mount(bsp_sdcard_t *ctx);

/**
 * @brief Unmount SD card
 *
 * @param[in] ctx     Pointer to SD card context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_unmount(bsp_sdcard_t *ctx);

/**
 * @brief Get SD card information
 *
 * @param[in]  ctx   Pointer to SD card context
 * @param[out] info  Pointer to info structure
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_get_info(bsp_sdcard_t *ctx, bsp_sdcard_info_t *info);

/**
 * @brief Get card type as string
 *
 * @param[in] type Card type enum
 *
 * @return const char* Card type string
 */
const char *bsp_sdcard_type_to_str(bsp_sdcard_type_t type);

/**
 * @brief Check if file exists
 *
 * @param[in] ctx   Pointer to SD card context
 * @param[in] path  File path
 *
 * @return true if file exists, false otherwise
 */
bool bsp_sdcard_file_exists(bsp_sdcard_t *ctx, const char *path);

/**
 * @brief Create a directory
 *
 * @param[in] ctx   Pointer to SD card context
 * @param[in] path  Directory path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_mkdir(bsp_sdcard_t *ctx, const char *path);

/**
 * @brief Remove a file
 *
 * @param[in] ctx   Pointer to SD card context
 * @param[in] path  File path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_remove(bsp_sdcard_t *ctx, const char *path);

/**
 * @brief Rename a file
 *
 * @param[in] ctx       Pointer to SD card context
 * @param[in] old_path  Old file path
 * @param[in] new_path  New file path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_rename(bsp_sdcard_t *ctx, const char *old_path, const char *new_path);

/**
 * @brief Write data to file
 *
 * @param[in] ctx      Pointer to SD card context
 * @param[in] path     File path
 * @param[in] data     Data buffer
 * @param[in] len      Data length
 * @param[in] append   Append mode (true) or overwrite (false)
 *
 * @return status_function_t Status of operation
 */
status_function_t
bsp_sdcard_write_file(bsp_sdcard_t *ctx, const char *path, const uint8_t *data, size_t len, bool append);

/**
 * @brief Write string to file
 *
 * @param[in] ctx      Pointer to SD card context
 * @param[in] path     File path
 * @param[in] str      String to write
 * @param[in] append   Append mode (true) or overwrite (false)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_write_string(bsp_sdcard_t *ctx, const char *path, const char *str, bool append);

/**
 * @brief Read data from file
 *
 * @param[in]  ctx      Pointer to SD card context
 * @param[in]  path     File path
 * @param[out] data     Data buffer
 * @param[in]  max_len  Maximum read length
 * @param[out] read_len Actual bytes read
 *
 * @return status_function_t Status of operation
 */
status_function_t
bsp_sdcard_read_file(bsp_sdcard_t *ctx, const char *path, uint8_t *data, size_t max_len, size_t *read_len);

/**
 * @brief Get file size
 *
 * @param[in]  ctx   Pointer to SD card context
 * @param[in]  path  File path
 * @param[out] size  File size in bytes
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_get_file_size(bsp_sdcard_t *ctx, const char *path, size_t *size);

/**
 * @brief List files in directory
 *
 * @param[in] ctx   Pointer to SD card context
 * @param[in] path  Directory path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_list_dir(bsp_sdcard_t *ctx, const char *path);

/**
 * @brief Run benchmark test
 *
 * @param[in]  ctx          Pointer to SD card context
 * @param[in]  rounds       Number of test rounds
 * @param[in]  block_size   Block size for testing
 * @param[in]  block_count  Number of blocks per round
 * @param[out] summary      Benchmark summary result
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_benchmark(bsp_sdcard_t                   *ctx,
                                       uint16_t                        rounds,
                                       uint16_t                        block_size,
                                       uint16_t                        block_count,
                                       bsp_sdcard_benchmark_summary_t *summary);

/**
 * @brief Quick write/read/verify test
 *
 * @param[in]  ctx     Pointer to SD card context
 * @param[out] result  Test result
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_quick_test(bsp_sdcard_t *ctx, bsp_sdcard_benchmark_result_t *result);

/**
 * @brief Check if SD card is mounted
 *
 * @param[in] ctx Pointer to SD card context
 *
 * @return true if mounted, false otherwise
 */
bool bsp_sdcard_is_mounted(bsp_sdcard_t *ctx);

/**
 * @brief Deinitialize SD card
 *
 * @param[in] ctx Pointer to SD card context
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_deinit(bsp_sdcard_t *ctx);

/* ═══════════════════════════════════════════════════════════════════════
 * CSV FILE FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief Create CSV file with header row
 *
 * @param[in] ctx      Pointer to SD card context
 * @param[in] path     File path (*.csv)
 * @param[in] headers  Array of column header strings
 * @param[in] num_cols Number of columns
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_csv_create(bsp_sdcard_t *ctx, const char *path, const char **headers, uint8_t num_cols);

/**
 * @brief Append a row of string values to CSV file
 *
 * @param[in] ctx      Pointer to SD card context
 * @param[in] path     File path (*.csv)
 * @param[in] values   Array of string values
 * @param[in] num_cols Number of columns
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_csv_append_row(bsp_sdcard_t *ctx, const char *path, const char **values, uint8_t num_cols);

/**
 * @brief Append a row of float values to CSV file
 *
 * @param[in] ctx       Pointer to SD card context
 * @param[in] path      File path (*.csv)
 * @param[in] values    Array of float values
 * @param[in] num_cols  Number of columns
 * @param[in] precision Decimal precision for float formatting
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_csv_append_floats(bsp_sdcard_t *ctx,
                                               const char   *path,
                                               const float  *values,
                                               uint8_t       num_cols,
                                               uint8_t       precision);

/**
 * @brief Append a row of integer values to CSV file
 *
 * @param[in] ctx      Pointer to SD card context
 * @param[in] path     File path (*.csv)
 * @param[in] values   Array of int32 values
 * @param[in] num_cols Number of columns
 *
 * @return status_function_t Status of operation
 */
status_function_t
bsp_sdcard_csv_append_ints(bsp_sdcard_t *ctx, const char *path, const int32_t *values, uint8_t num_cols);

/**
 * @brief Append a timestamped row to CSV file (timestamp + values)
 *
 * @param[in] ctx       Pointer to SD card context
 * @param[in] path      File path (*.csv)
 * @param[in] timestamp Timestamp string (e.g., "2026-03-07 12:00:00")
 * @param[in] values    Array of float values
 * @param[in] num_cols  Number of value columns
 * @param[in] precision Decimal precision for float formatting
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_csv_append_timestamped(bsp_sdcard_t *ctx,
                                                    const char   *path,
                                                    const char   *timestamp,
                                                    const float  *values,
                                                    uint8_t       num_cols,
                                                    uint8_t       precision);

/* ═══════════════════════════════════════════════════════════════════════
 * LOG FILE FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief Log level enumeration
 */
typedef enum
{
  BSP_SDCARD_LOG_DEBUG = 0,
  BSP_SDCARD_LOG_INFO,
  BSP_SDCARD_LOG_WARNING,
  BSP_SDCARD_LOG_ERROR,
  BSP_SDCARD_LOG_CRITICAL,
} bsp_sdcard_log_level_t;

/**
 * @brief Create log file with header
 *
 * @param[in] ctx         Pointer to SD card context
 * @param[in] path        File path (*.log)
 * @param[in] app_name    Application name for log header
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_log_create(bsp_sdcard_t *ctx, const char *path, const char *app_name);

/**
 * @brief Write a log entry with level and message
 *
 * @param[in] ctx     Pointer to SD card context
 * @param[in] path    File path (*.log)
 * @param[in] level   Log level
 * @param[in] message Log message
 *
 * @return status_function_t Status of operation
 */
status_function_t
bsp_sdcard_log_write(bsp_sdcard_t *ctx, const char *path, bsp_sdcard_log_level_t level, const char *message);

/**
 * @brief Write a timestamped log entry
 *
 * @param[in] ctx       Pointer to SD card context
 * @param[in] path      File path (*.log)
 * @param[in] timestamp Timestamp string
 * @param[in] level     Log level
 * @param[in] message   Log message
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_log_write_timestamped(bsp_sdcard_t          *ctx,
                                                   const char            *path,
                                                   const char            *timestamp,
                                                   bsp_sdcard_log_level_t level,
                                                   const char            *message);

/**
 * @brief Write a formatted log entry (printf-style)
 *
 * @param[in] ctx    Pointer to SD card context
 * @param[in] path   File path (*.log)
 * @param[in] level  Log level
 * @param[in] format Printf format string
 * @param[in] ...    Variable arguments
 *
 * @return status_function_t Status of operation
 */
status_function_t
bsp_sdcard_log_printf(bsp_sdcard_t *ctx, const char *path, bsp_sdcard_log_level_t level, const char *format, ...);

/**
 * @brief Get log level as string
 *
 * @param[in] level Log level enum
 *
 * @return const char* Log level string
 */
const char *bsp_sdcard_log_level_to_str(bsp_sdcard_log_level_t level);

#endif /*End file _BSP_SDCARD_H_*/

/* End of file -------------------------------------------------------- */
