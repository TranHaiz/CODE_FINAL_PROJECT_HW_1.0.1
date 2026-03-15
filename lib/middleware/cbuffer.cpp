/**
 * @file       cbuffer.c
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2025-07-21
 * @author     Hai Tran
 * @brief      This is a circular buffer (ring buffer) implementation in C.
 *             It supports storing and retrieving data in a FIFO manner,
 *             suitable for embedded systems, producers/consumers, streaming, etc.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "cbuffer.h"

#include "common_type.h"

/* Private defines ---------------------------------------------------- */
#define READ_VER_1
#define WRITE_VER_1

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */
/**
 * @brief      Debug log
 *
 * @attention  Print debug log if DEBUG is define
 *
 * @return     None
 */
#ifdef DEBUG
#define DEBUG_LOG(msg) fprintf(stderr, "[DEBUG] %s(): %s (line %d)\n", __func__, msg, __LINE__)
#else
#define DEBUG_LOG(msg)
#endif
// #define INITIALIZE_WRITER_READER_OVERFLOW

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void cb_init(cbuffer_t *cb, void *buf, uint32_t size)
{
  // Initial condition check
  if (cb == NULL || buf == NULL || size == 0 || size > CB_MAX_SIZE)
  {
    DEBUG_LOG("Cbuffer is invalid, initialization failed");
    return;
  }

  cb->data     = (uint8_t *) buf;
  cb->size     = size;
  cb->writer   = 0;
  cb->reader   = 0;
  cb->overflow = 0;
  cb->active   = true;
}

void cb_clear(cbuffer_t *cb)
{
  // Check if pointer is empty. If yes, display message log
  if (cb == NULL)
  {
    DEBUG_LOG("Cbuffer is invalid, clearance failed");
    return;
  }

  // Method 1: Reset all indices and counter
  //           The old data (read or not) is not explicitly erased
  //           Treat cbuffer as empty (although not)

  cb->writer   = 0;  // Set the writer index to 0
  cb->reader   = 0;  // Set the reader index to 0
  cb->overflow = 0;  // Set the overflow counter to 0
}

#ifdef READ_VER_1
uint32_t cb_read(cbuffer_t *cb, void *buf, uint32_t nbytes)
{
  // Validate input parameters.
  if ((!cb) || (!buf) || (cb->active != 1))
  {
    DEBUG_LOG("Can't read");
    return false;
  }

  // Cast the destination buffer to byte pointer.
  uint8_t *r_buffer = (uint8_t *) buf;
  uint32_t r_index;

  // Read up to nbytes from the circular buffer.
  for (r_index = 0; r_index < nbytes; r_index++)
  {
    // If reader equals writer, buffer is empty.
    if (cb->reader == cb->writer)
    {
      DEBUG_LOG("No data to read from buffer");
      break;
    }
    // Read one byte from buffer and store it in the output buffer.
    r_buffer[r_index] = cb->data[cb->reader];
    // Advance the reader index, wrap around if needed.
    cb->reader = (cb->reader + 1) % cb->size;
  }

  return r_index;
}
#endif

#ifdef WRITE_VER_1
uint32_t cb_write(cbuffer_t *cb, void *buf, uint32_t nbytes)
{
  uint32_t w_index;                     // Write index
  uint8_t *w_buffer = (uint8_t *) buf;  // Write buffer

  // Check condition
  if (cb == NULL || buf == NULL || cb->active != 1)
  {
    DEBUG_LOG("Can't write");
    return false;
  }

  if (nbytes <= 0 || nbytes > CB_MAX_SIZE || nbytes >= cb->size)
  {
    DEBUG_LOG("Can't write");
    return false;
  }

  // Write data from buff
  for (w_index = 0; w_index < nbytes; w_index++)
  {
    if (cb_space_count(cb) == 0)  // check valid space
    {
      DEBUG_LOG("Don't have enough space");
      break;
    }
    // Write data
    cb->data[cb->writer] = w_buffer[w_index];
    // Update writer
    cb->writer = (cb->writer == cb->size - 1) ? 0 : cb->writer + 1;
  }
  // Update overflow
  cb->overflow += nbytes - w_index;

  return w_index;
}
#endif

uint32_t cb_data_count(cbuffer_t *cb)
{
  // Check for NULL cb struct pointer (invalid buffer reference)
  if ((cb == NULL))
  {
    DEBUG_LOG("Circular buffer struct pointer is NULL");
    return false;
  }

  // Check if cb struct has been initialized with cb_init function
  else if (cb->active == false)
  {
    DEBUG_LOG("Uninitialized circular buffer struct");
    return false;
  }

  // Calculate the readable bytes
  if ((cb->reader) <= (cb->writer))
  {
    return ((cb->writer) - (cb->reader));
  }
  else
  {
    return ((cb->writer) + (cb->size) - (cb->reader));
  }
}

uint32_t cb_space_count(cbuffer_t *cb)
{
  // Check for NULL cb struct pointer (invalid buffer reference)
  if ((cb == NULL))
  {
    DEBUG_LOG("Circular buffer pointer is NULL");
    return false;
  }

  // Check if cb struct has been initialized with cb_init function
  else if (cb->active == false)
  {
    DEBUG_LOG("Uninitialized circular buffer struct");
    return false;
  }

  // Calculate the spaces can be written
  return ((cb->size) - (cb_data_count(cb)) - 1);
}
/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */