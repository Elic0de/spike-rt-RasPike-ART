#ifndef ETROBO_FIRMWARE_APP_FIRMWARE_APPLICATION_H
#define ETROBO_FIRMWARE_APP_FIRMWARE_APPLICATION_H

#include "comm/SerialProtocol.h"
#include "mission/MissionManager.h"

namespace etrobo {
namespace app {

class FirmwareApplication {
 public:
  void initialize();
  void runOneCycle();

 private:
  comm::SerialProtocol serial_{};
  mission::MissionManager mission_{};

  void processSerialCommands();
};

}  // namespace app
}  // namespace etrobo

#endif
