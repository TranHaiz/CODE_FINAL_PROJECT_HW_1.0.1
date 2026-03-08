/**
 * @file       cbuffer.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date
 * @author     Hai Tran
 * @brief      Circular Buffer
 *             This Circular Buffer is safe to use in IRQ with single reader,
 *             single writer. No need to disable any IRQ.
 *
 *             Capacity = <size> - 1
 * @note       None
 * @example    cbuffer_t cb;
 *             uint8_t cb_buff[6];
 *             uint8_t a;
 *             void main(void)
 *             {
 *                 cb_init(&cb, cb_buff, 100);
 *                 cb_clear(&cb);
 *                 char a[] = {0, 1, 2, 3, 4};
 *
 *                 int c = 5;
 *                 cb_write(&cb, a, 5);
 *                 char b[5];
 *                 cb_read(&cb, a, 5);
 *             }
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __CBUFFER_H
#define __CBUFFER_H

/* Includes ----------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Public defines ----------------------------------------------------- */
#define CB_MAX_SIZE (0x00800000) /*Set the maximum size of CB to 8388608 bytes (decimal)*/

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  uint8_t *data;     /*Pointer to cbuffer in memory*/
  size_t   size;     /*Total size of cbuffer*/
  size_t   writer;   /*Write index: Keep track when the next data is written*/
  size_t   reader;   /*Read index: Keep track when the next data is read*/
  size_t   overflow; /*Overflow counter: The number of failed write attempts*/
  bool     active;   /*Flag to check to status of the cbuffer*/
} cbuffer_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief      Initialize circular buffer
 *
 * @param[in]  cb   Pointer to cbuffer_t variable of struct
 * @param[in]  buf  Data write to cbuffer
 * @param[in]  size Input size of circular buffer
 *
 * @attention  All Cbuffer must be initialized before any kind of action involved that certain buffer.
 *
 * @return     None
 */
void cb_init(cbuffer_t *cb, void *buf, size_t size);

/**
 * @brief      Clear counter and data assigned to buffer
 *
 * @details    Reset Cbuffer’s reader, writer, and overflow counter
 *             Technically, ignore and allow overwrite of all currently valid data in Cbuffer.
 *
 * @param[in]  cb   Pointer to cbuffer_t variable of struct
 * @param[in]  buf  Data write to cbuffer
 * @param[in]  size Input size of circular buffer
 *
 * @return     None
 */
void cb_clear(cbuffer_t *cb);

/**
 * @brief  Read data from cbuffer
 *
 * @details  Copy data from cbuffer to destination buffer.
 *           Stop if no more data to read or nbytes reached.
 *
 * @param[in]   cb      Pointer to cbuffer_t variable
 * @param[out]  buf     Pointer to destination buffer
 * @param[in]   nbytes  Number of bytes to read
 *
 * @attention  Init before use this function
 *
 * @return  Number of bytes actually read from cbuffer
 */
size_t cb_read(cbuffer_t *cb, void *buf, size_t nbytes);

/**
 * @brief      Write data
 *
 * @details    Write a number of byte to Cbuffer and raise the writer count
 *
 * @param[in]  cb   Pointer to cbuffer_t varriable
 * @param[in]  buf  Data write to cbuffer
 * @param[in]  size Size of written data
 *
 * @attention  Init before use this function
 *
 * @return     The actual written bytes

 */
size_t cb_write(cbuffer_t *cb, void *buf, size_t nbytes);

/**
 * @brief  Count readable bytes from cbuffer
 *
 * @details  Calculate the number of readable bytes
 *           base on reader and writer count
 *
 * @param[in]   cb  Pointer to cbuffer_t varriable
 *
 * @attention  Init before use this function
 *
 * @return  The number of readable bytes from cbuffer
 *
 */
size_t cb_data_count(cbuffer_t *cb);

/**
 * @brief  Count data spaces can be written to cbuffer
 *
 * @details  Use the size of cbuffer minus the number of readable bytes
 *           to determine the remaining space
 *
 * @param[in]   cb  Pointer to cbuffer_t varriable
 *
 * @attention  Init before use this function
 *
 * @return  The number of data spaces can be written to cbuffer
 *
 */
size_t cb_space_count(cbuffer_t *cb);
#endif  // __CBUFFER_H

/* End of file -------------------------------------------------------- */
