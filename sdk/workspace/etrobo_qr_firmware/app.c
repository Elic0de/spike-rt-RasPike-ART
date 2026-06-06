#include "app.h"
#include "line_controller.h"
#include "protocol.h"
#include "serial_protocol.h"

#include <stdint.h>

static void send_status(void)
{
  LineStatus status;

  line_controller_get_status(&status);
  serial_protocol_send_status(&status);
}

static void handle_command(uint8_t cmd, const char *payload, uint8_t size)
{
  switch (cmd) {
    case APP_CMD_CONFIG:
      serial_protocol_send_ack(cmd, (size >= sizeof(LineConfig)) ?
          line_controller_configure((const LineConfig *)payload) : 0);
      break;
    case APP_CMD_START:
      serial_protocol_send_ack(cmd, line_controller_start());
      break;
    case APP_CMD_STOP:
      line_controller_stop();
      serial_protocol_send_ack(cmd, 1);
      break;
    case APP_CMD_QR_CORRECTION:
      serial_protocol_send_ack(cmd, (size >= sizeof(QrCorrection)) ?
          line_controller_apply_qr((const QrCorrection *)payload) : 0);
      break;
    case APP_CMD_STATUS:
      send_status();
      break;
    default:
      serial_protocol_send_ack(cmd, 0);
      break;
  }
}

void main_task(intptr_t exinf)
{
  uint8_t cmd;
  uint8_t size;
  char payload[128];

  serial_protocol_init();
  dly_tsk(1000 * 1000);

  line_controller_configure_default();

  for (;;) {
    if (serial_protocol_receive(&cmd, &size, payload, sizeof(payload))) {
      handle_command(cmd, payload, size);
    }
  }
}

void line_tracer_task(intptr_t exinf)
{
  line_controller_run_cycle();
  ext_tsk();
}
