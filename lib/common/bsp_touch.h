/**
 * @file       bsp_touch.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    0.1.0
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      Simple touch controller wrapper sharing the TFT SPI bus.
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_TOUCH_H_
#define _BSP_TOUCH_H_
/* Includes ----------------------------------------------------------- */
#include <Arduino.h>
#include <TFT_eSPI.h>

/* Public defines ----------------------------------------------------- */
typedef struct
{
  TFT_eSPI *tft;
  bool      invert_x;
  bool      invert_y;
} bsp_touch_t;

typedef struct
{
  bool     touched;
  uint16_t x;
  uint16_t y;
} bsp_touch_point_t;

/* Public function prototypes ----------------------------------------- */
void bsp_touch_init(bsp_touch_t *ctx, TFT_eSPI *driver);
bool bsp_touch_read(bsp_touch_t *ctx, bsp_touch_point_t *point);
bool bsp_touch_is_pressed(void);

#endif /*End file _BSP_TOUCH_H_*/

/* End of file -------------------------------------------------------- */
