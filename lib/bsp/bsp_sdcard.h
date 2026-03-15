/**
 * @file       bsp_sdcard.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.1.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      Simplified BSP for SD Card
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
/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief File open mode
 */
typedef enum
{
  BSP_SDCARD_MODE_READ,   // Open for reading
  BSP_SDCARD_MODE_WRITE,  // Create/truncate and write
  BSP_SDCARD_MODE_APPEND  // Open for appending
} bsp_sdcard_mode_t;

/**
 * @brief SD card file handle structure
 */
typedef struct
{
  File   file;      // Arduino File object
  bool   is_open;   // File open status
  size_t position;  // Current position in file
} bsp_sdcard_file_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize and mount SD card
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_init(void);

/**
 * @brief Check if SD card is mounted
 */
status_function_t bsp_sdcard_is_mounted(void);

/**
 * @brief Open file with specified mode
 *
 * @param[in]  path  File path
 * @param[in]  mode  Open mode (READ, WRITE, APPEND)
 * @param[out] file  Pointer to file handle
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_open(const char *path, bsp_sdcard_mode_t mode, bsp_sdcard_file_t *file);

/**
 * @brief Seek to position in file
 *
 * @param[in] file     Pointer to file handle
 * @param[in] position Position to seek to (from beginning)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_seek(bsp_sdcard_file_t *file, size_t position);

/**
 * @brief Write data to file at current position
 *
 * @param[in]  file         Pointer to file handle
 * @param[in]  data         Data buffer to write
 * @param[in]  len          Number of bytes to write
 * @param[out] written_len  Actual bytes written (can be NULL)
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_write(bsp_sdcard_file_t *file, const uint8_t *data, size_t len, size_t *written_len);

/**
 * @brief Read data from file at current position
 *
 * @param[in]  file     Pointer to file handle
 * @param[out] data     Buffer to read into
 * @param[in]  max_len  Maximum bytes to read
 * @param[out] read_len Actual bytes read
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_read(bsp_sdcard_file_t *file, uint8_t *data, size_t max_len, size_t *read_len);

/**
 * @brief Close file handle
 *
 * @param[in] file Pointer to file handle
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_close(bsp_sdcard_file_t *file);

/**
 * @brief Delete a file or directory
 *
 * @param[in] path File or directory path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_delete(const char *path);

/**
 * @brief Rename a file or directory
 *
 * @param[in] old_path Current path
 * @param[in] new_path New path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_rename(const char *old_path, const char *new_path);

/**
 * @brief Create a directory
 *
 * @param[in] path Directory path
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_mkdir(const char *path);

/**
 * @brief Deinitialize SD card
 *
 * @return status_function_t Status of operation
 */
status_function_t bsp_sdcard_deinit(void);

#endif /*End file _BSP_SDCARD_H_*/

/* End of file -------------------------------------------------------- */
