/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                    Graduate School of Information Science, Nagoya Univ., JAPAN
 */

#ifndef __RASPIKE_H__
#define __RASPIKE_H__
#include <kernel.h>

/*
 *  タスクの優先度の定義
 */

#define APP_HIGHEST_PRIORITY 3
#define MAIN_PRIORITY   5    /* メインタスクの優先度 */

/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#define MAIN_STACK_SIZE 4096    /* タスクのスタックサイズ */
#define CRITICAL_TX_STACK_SIZE 1024

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void notify_task(intptr_t exinf);
extern void drive_telemetry_task(intptr_t exinf);
extern void soner_task(intptr_t exinf);
extern void realtime_drive_task(intptr_t exinf);
extern void realtime_drive_cyclic_handler(intptr_t exinf);
extern void critical_tx_task(intptr_t exinf);
extern void safety_supervisor_task(intptr_t exinf);
extern void safety_supervisor_cyclic_handler(intptr_t exinf);
extern void runtime_telemetry_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#endif /* __RASPIKE_H__ */
