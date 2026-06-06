#ifndef ETROBO_FIRMWARE_LIB_CONTROL_LINE_CONTROLLER_H
#define ETROBO_FIRMWARE_LIB_CONTROL_LINE_CONTROLLER_H

#include "protocol.h"

#include <cstdint>

#include <kernel.h>
#include <spike/pup/colorsensor.h>
#include <spike/pup/motor.h>

namespace etrobo {
namespace control {

class LineController {
 public:
  void configureDefault();
  int configure(const LineConfig* config);
  int start();
  void stop();
  int applyQr(const QrCorrection* correction);
  void getStatus(LineStatus& status) const;
  void runCycle();

 private:
  struct ControlState {
    bool configured = false;
    bool running = false;
    LineConfig config{};
    QrCorrection correction{};
    SYSTIM correctionUntil = 0;
    uint32_t lastQrSeq = 0;
    float integral = 0.0F;
    float previousError = 0.0F;
    pup_device_t* colorSensor = nullptr;
    pup_motor_t* leftMotor = nullptr;
    pup_motor_t* rightMotor = nullptr;
    LineStatus status{};
  };

  ControlState state_{};

  static const LineConfig& defaultConfig();
  static int32_t clampI32(int32_t value, int32_t minValue, int32_t maxValue);
  static float clampFloat(float value, float minValue, float maxValue);
  static bool validPort(uint8_t port);
};

}  // namespace control
}  // namespace etrobo

#endif
