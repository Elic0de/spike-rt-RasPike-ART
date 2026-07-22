// SPDX-License-Identifier: MIT
/*
 * API for the hub built-in status light.
 *
 * Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                    Graduate School of Information Science, Nagoya Univ., JAPAN
 */

/**
 * \file    spike/hub/light.h
 * \brief   API for the hub built-in status light.
 * \author  Shu Yoshifumi
 */

/**
 * \addtogroup  Hub Hub
 * @{
 */


/**
 * \~English
 * \defgroup  System System
 * \brief     API for the entire hub system.
 * @{
 *
 * \~Japanese
 * \defgroup  System システム
 * \brief     ハブ全体のシステム向けAPI．
 * @{
 */


#ifndef _HUB_SYSTEM_H_
#define _HUB_SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * \~English
 * \brief       Shut down the system.
 * \details     Equivalent to exit(0). Do not call in CPU locked state.
 *
 * \~Japanese
 * \brief       システムをシャットダウンさせる．
 * \details     標準ライブラリexit(0)と同じ．CPUロック状態から呼び出してはいけない．
 */
void hub_system_shutdown(void);

/** Stops all Pybricks motor controllers and then brakes or coasts every H-bridge. */
void hub_emergency_stop_all(bool brake);

/** Fault-context-safe drive disable. Does not allocate, lock, or call the RTOS. */
void hub_motor_emergency_disable_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif // _HUB_SYSTEM_H_

/**
 * @} // End of group System
 */

/**
 * @} // End of group Hub
 */
