#ifndef ETROBO_QR_FIRMWARE_LINE_CONTROLLER_H
#define ETROBO_QR_FIRMWARE_LINE_CONTROLLER_H

#include "protocol.h"

void line_controller_configure_default(void);
int line_controller_configure(const LineConfig *config);
int line_controller_start(void);
void line_controller_stop(void);
int line_controller_apply_qr(const QrCorrection *correction);
void line_controller_get_status(LineStatus *status);
void line_controller_run_cycle(void);

#endif
