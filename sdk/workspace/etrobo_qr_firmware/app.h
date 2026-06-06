#ifndef ETROBO_QR_FIRMWARE_APP_H
#define ETROBO_QR_FIRMWARE_APP_H

#include <kernel.h>

#define MAIN_PRIORITY        5
#define TRACER_PRIORITY      4
#define APP_HIGHEST_PRIORITY 4

#define STACK_SIZE           4096
#define LINE_TRACER_PERIOD   (10 * 1000)

#ifndef TOPPERS_MACRO_ONLY
extern void main_task(intptr_t exinf);
extern void line_tracer_task(intptr_t exinf);
#endif

#endif
