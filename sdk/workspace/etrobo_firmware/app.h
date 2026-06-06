#ifndef ETROBO_FIRMWARE_APP_H
#define ETROBO_FIRMWARE_APP_H

#include <kernel.h>

#define MAIN_PRIORITY        5
#define CONTROL_PRIORITY     4
#define APP_HIGHEST_PRIORITY 4

#define STACK_SIZE           4096
#define CONTROL_PERIOD       (10 * 1000)

#ifndef TOPPERS_MACRO_ONLY
#ifdef __cplusplus
extern "C" {
#endif
extern void main_task(intptr_t exinf);
extern void control_task(intptr_t exinf);
#ifdef __cplusplus
}
#endif
#endif

#endif
