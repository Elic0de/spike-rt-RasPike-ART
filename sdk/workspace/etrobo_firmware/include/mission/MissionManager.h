#ifndef ETROBO_FIRMWARE_MISSION_MISSION_MANAGER_H
#define ETROBO_FIRMWARE_MISSION_MISSION_MANAGER_H

#include "lib/control/LineController.h"
#include "protocol.h"

namespace etrobo {
namespace mission {

class MissionManager {
 public:
  void initialize();
  int configureLine(const LineConfig* config);
  int start();
  void stop();
  int applyQr(const QrCorrection* correction);
  void getStatus(LineStatus& status) const;
  void runControlCycle();

 private:
  control::LineController lineController_{};
};

}  // namespace mission
}  // namespace etrobo

#endif
