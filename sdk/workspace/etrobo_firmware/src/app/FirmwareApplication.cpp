#include "app/FirmwareApplication.h"

#include "protocol.h"

namespace etrobo {
namespace app {

void FirmwareApplication::initialize()
{
  serial_.initialize();
  dly_tsk(1000 * 1000);
  mission_.initialize();
}

void FirmwareApplication::runOneCycle()
{
  processSerialCommands();
  mission_.runControlCycle();
}

void FirmwareApplication::processSerialCommands()
{
  uint8_t cmd;
  uint8_t size;
  char payload[128];

  while (serial_.receive(cmd, size, payload, sizeof(payload), false)) {
    switch (cmd) {
      case APP_CMD_CONFIG:
        serial_.sendAck(cmd, (size >= sizeof(LineConfig)) ?
            mission_.configureLine(reinterpret_cast<const LineConfig*>(payload)) : 0);
        break;
      case APP_CMD_START:
        serial_.sendAck(cmd, mission_.start());
        break;
      case APP_CMD_STOP:
        mission_.stop();
        serial_.sendAck(cmd, 1);
        break;
      case APP_CMD_QR_CORRECTION:
        serial_.sendAck(cmd, (size >= sizeof(QrCorrection)) ?
            mission_.applyQr(reinterpret_cast<const QrCorrection*>(payload)) : 0);
        break;
      case APP_CMD_STATUS: {
        LineStatus status;
        mission_.getStatus(status);
        serial_.sendStatus(status);
        break;
      }
      default:
        serial_.sendAck(cmd, 0);
        break;
    }
  }
}

}  // namespace app
}  // namespace etrobo
