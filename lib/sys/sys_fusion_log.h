/**
 * @file       sys_fusion_log.h
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-24
 * @author     Hai Tran
 *
 * @brief      Dedicated CSV log of fusion data to SD card (non-blocking producer)
 *
 */

#ifndef _SYS_FUSION_LOG_H_
#define _SYS_FUSION_LOG_H_

#include "common_type.h"
#include "device_config.h"

#if (DEVICE_FUSION_DEBUG_LOG_ENABLED == 1)

#include "sys_fusion.h"

/**
 * @brief Initialize fusion log (create CSV file with header)
 */
void sys_fusion_log_init(void);

/**
 * @brief Push one fusion sample into ring buffer. Non-blocking — drops if buffer full.
 * @param[in] data Fusion data (must have DEVICE_FUSION_DEBUG_MODE fields populated)
 * @param[in] t_ms Timestamp (millis)
 */
void sys_fusion_log_push(const sys_fusion_data_t *data, size_t t_ms);

/**
 * @brief Drain ring buffer to SD card if threshold hit or interval elapsed.
 *        Call from low-priority logging thread.
 */
void sys_fusion_log_process(void);

/**
 * @brief Force flush all buffered data to SD card now (blocking).
 *        Use before stopping a test session to ensure no data loss.
 */
void sys_fusion_log_flush(void);

#endif /* DEVICE_FUSION_DEBUG_LOG_ENABLED */

#endif /*End file _SYS_FUSION_LOG_H_*/

/* End of file -------------------------------------------------------- */
