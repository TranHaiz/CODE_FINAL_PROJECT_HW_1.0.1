/**
 * @file       fifo.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-12
 * @author     Hai Tran
 *
 * @brief     FIFO (First-In-First-Out) implementation
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _FIFO_H_
#define _FIFO_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  uint8_t *buffer;
  size_t   item_size;
  size_t   capacity;
  size_t   head;
  size_t   tail;
  size_t   count;
} fifo_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief  Initialize FIFO with caller-provided buffer
 *
 * @param[in] fifo       Pointer to fifo_t instance
 * @param[in] buffer     Raw buffer to use as storage
 * @param[in] item_size  Size of each item in bytes
 * @param[in] capacity   Maximum number of items the buffer can hold
 */
static inline void fifo_init(fifo_t *fifo, void *buffer, size_t item_size, size_t capacity)
{
  fifo->buffer    = (uint8_t *) buffer;
  fifo->item_size = item_size;
  fifo->capacity  = capacity;
  fifo->head      = 0;
  fifo->tail      = 0;
  fifo->count     = 0;
}

/**
 * @brief  Push one item into the FIFO
 *
 * @param[in] fifo  Pointer to fifo_t instance
 * @param[in] item  Pointer to item to copy in
 *
 * @return true if pushed, false if full
 */
static inline bool fifo_push(fifo_t *fifo, const void *item)
{
  if (fifo->count >= fifo->capacity)
  {
    return false;
  }

  memcpy(fifo->buffer + fifo->tail * fifo->item_size, item, fifo->item_size);
  fifo->tail = (fifo->tail + 1) % fifo->capacity;
  fifo->count++;

  return true;
}

/**
 * @brief  Pop one item from the FIFO
 *
 * @param[in]  fifo  Pointer to fifo_t instance
 * @param[out] item  Pointer to buffer to copy item into
 *
 * @return true if popped, false if empty
 */
static inline bool fifo_pop(fifo_t *fifo, void *item)
{
  if (fifo->count == 0)
  {
    return false;
  }

  memcpy(item, fifo->buffer + fifo->head * fifo->item_size, fifo->item_size);
  fifo->head = (fifo->head + 1) % fifo->capacity;
  fifo->count--;

  return true;
}

/**
 * @brief  Peek at the front item without removing it
 *
 * @param[in]  fifo  Pointer to fifo_t instance
 * @param[out] item  Pointer to buffer to copy item into
 *
 * @return true if peeked, false if empty
 */
static inline bool fifo_peek(fifo_t *fifo, void *item)
{
  if (fifo->count == 0)
  {
    return false;
  }

  memcpy(item, fifo->buffer + fifo->head * fifo->item_size, fifo->item_size);

  return true;
}

/**
 * @brief  Check if FIFO is empty
 */
static inline bool fifo_is_empty(const fifo_t *fifo)
{
  return (fifo->count == 0);
}

/**
 * @brief  Check if FIFO is full
 */
static inline bool fifo_is_full(const fifo_t *fifo)
{
  return (fifo->count >= fifo->capacity);
}

/**
 * @brief  Get current number of items in FIFO
 */
static inline size_t fifo_count(const fifo_t *fifo)
{
  return fifo->count;
}

/**
 * @brief  Reset FIFO to empty state without clearing buffer
 */
static inline void fifo_reset(fifo_t *fifo)
{
  fifo->head  = 0;
  fifo->tail  = 0;
  fifo->count = 0;
}

#endif /*End file _FIFO_H_*/

/* End of file -------------------------------------------------------- */