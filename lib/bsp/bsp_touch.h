/**
 * @file       bsp_touch.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-03-08
 * @author     Hai Tran
 *
 * @brief      Simple touch controller wrapper (Singleton)
 *
 * @note       Shares the TFT SPI bus via bsp_display singleton.
 *             Only supports single instance.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_TOUCH_H_
#define _BSP_TOUCH_H_

/* Includes ----------------------------------------------------------- */
#include <Arduino.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */

/**
 * @brief Touch point data structure
 */
typedef struct
{
  bool     touched;
  uint16_t x;
  uint16_t y;
} bsp_touch_point_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize touch controller (requires bsp_display_init first)
 */
void bsp_touch_init(void);

/**
 * @brief Read touch point
 *
 * @param[out] point Pointer to touch point structure
 *
 * @return true if touch detected, false otherwise
 */
bool bsp_touch_read(bsp_touch_point_t *point);

/**
 * @brief Check if touch is currently pressed (via IRQ pin if available)
 */
bool bsp_touch_is_pressed(void);

#endif /* _BSP_TOUCH_H_ */

/* End of file -------------------------------------------------------- */
