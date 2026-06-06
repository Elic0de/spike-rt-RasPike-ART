#include "lib/control/LineController.h"

#include "kernel_cfg.h"

#include <cstring>

#include <pbio/error.h>

namespace etrobo {
namespace control {

namespace {

constexpr int32_t kMaxDevices = 6;
constexpr int32_t kMaxPower = 100;
constexpr uint32_t kMaxQrValidUs = 5 * 1000 * 1000;

constexpr pbio_port_id_t portFromIndex(uint8_t port)
{
  return static_cast<pbio_port_id_t>(port + 'A');
}

}  // namespace

const LineConfig& LineController::defaultConfig()
{
  static const LineConfig config = {
    4,
    1,
    0,
    1,
    PUP_DIRECTION_COUNTERCLOCKWISE,
    PUP_DIRECTION_CLOCKWISE,
    1,
    0,
    45,
    30,
    60,
    50,
    0.4F,
    0.0F,
    0.0F,
    1000.0F,
  };
  return config;
}

void LineController::configureDefault()
{
  configure(&defaultConfig());
}

int LineController::configure(const LineConfig* config)
{
  if (config == nullptr ||
      !validPort(config->color_port) ||
      !validPort(config->left_motor_port) ||
      !validPort(config->right_motor_port)) {
    return 0;
  }

  pup_device_t* colorSensor = pup_color_sensor_get_device(portFromIndex(config->color_port));
  pup_motor_t* leftMotor = pup_motor_get_device(portFromIndex(config->left_motor_port));
  pup_motor_t* rightMotor = pup_motor_get_device(portFromIndex(config->right_motor_port));
  if (colorSensor == nullptr || leftMotor == nullptr || rightMotor == nullptr) {
    return 0;
  }

  pup_direction_t leftDirection =
      (config->left_direction == PUP_DIRECTION_COUNTERCLOCKWISE) ?
      PUP_DIRECTION_COUNTERCLOCKWISE : PUP_DIRECTION_CLOCKWISE;
  pup_direction_t rightDirection =
      (config->right_direction == PUP_DIRECTION_COUNTERCLOCKWISE) ?
      PUP_DIRECTION_COUNTERCLOCKWISE : PUP_DIRECTION_CLOCKWISE;
  bool resetCount = config->reset_count != 0;
  if (pup_motor_setup(leftMotor, leftDirection, resetCount) != PBIO_SUCCESS ||
      pup_motor_setup(rightMotor, rightDirection, resetCount) != PBIO_SUCCESS) {
    return 0;
  }

  LineConfig normalized = *config;
  normalized.edge = (normalized.edge < 0) ? -1 : 1;
  normalized.max_power = clampI32(normalized.max_power, 1, kMaxPower);
  normalized.base_power = clampI32(normalized.base_power, -normalized.max_power,
                                   normalized.max_power);
  normalized.turn_limit = clampI32(normalized.turn_limit, 1, normalized.max_power);
  if (normalized.integral_limit <= 0.0F) {
    normalized.integral_limit = 1000.0F;
  }

  loc_mtx(CONTROL_MUTEX);
  state_.config = normalized;
  state_.colorSensor = colorSensor;
  state_.leftMotor = leftMotor;
  state_.rightMotor = rightMotor;
  state_.configured = true;
  state_.running = false;
  state_.integral = 0.0F;
  state_.previousError = 0.0F;
  std::memset(&state_.correction, 0, sizeof(state_.correction));
  std::memset(&state_.status, 0, sizeof(state_.status));
  state_.status.configured = 1;
  unl_mtx(CONTROL_MUTEX);

  return 1;
}

int LineController::start()
{
  int ok;

  loc_mtx(CONTROL_MUTEX);
  ok = state_.configured ? 1 : 0;
  if (ok) {
    state_.running = true;
    state_.integral = 0.0F;
    state_.previousError = 0.0F;
    state_.status.running = 1;
  }
  unl_mtx(CONTROL_MUTEX);

  return ok;
}

void LineController::stop()
{
  pup_motor_t* leftMotor;
  pup_motor_t* rightMotor;

  loc_mtx(CONTROL_MUTEX);
  state_.running = false;
  state_.status.running = 0;
  leftMotor = state_.leftMotor;
  rightMotor = state_.rightMotor;
  unl_mtx(CONTROL_MUTEX);

  if (leftMotor != nullptr) {
    pup_motor_stop(leftMotor);
  }
  if (rightMotor != nullptr) {
    pup_motor_stop(rightMotor);
  }
}

int LineController::applyQr(const QrCorrection* correction)
{
  SYSTIM now;

  if (correction == nullptr) {
    return 0;
  }
  get_tim(&now);

  loc_mtx(CONTROL_MUTEX);
  if (correction->seq <= state_.lastQrSeq) {
    unl_mtx(CONTROL_MUTEX);
    return 0;
  }

  state_.correction = *correction;
  state_.correction.base_power_delta = clampI32(correction->base_power_delta, -30, 30);
  state_.correction.target_reflection_delta =
      clampI32(correction->target_reflection_delta, -30, 30);
  state_.correction.turn_bias = clampI32(correction->turn_bias, -30, 30);
  uint32_t validUs = correction->valid_ms * 1000;
  if (validUs == 0 || validUs > kMaxQrValidUs) {
    validUs = kMaxQrValidUs;
  }
  state_.correctionUntil = now + validUs;
  state_.lastQrSeq = correction->seq;
  state_.status.qr_seq = correction->seq;
  state_.status.qr_id = correction->qr_id;
  state_.status.mode = correction->mode;
  state_.status.qr_active = 1;
  unl_mtx(CONTROL_MUTEX);

  return 1;
}

void LineController::getStatus(LineStatus& status) const
{
  loc_mtx(CONTROL_MUTEX);
  status = state_.status;
  status.configured = state_.configured ? 1 : 0;
  status.running = state_.running ? 1 : 0;
  unl_mtx(CONTROL_MUTEX);
}

void LineController::runCycle()
{
  bool running;
  bool qrActive;
  LineConfig config;
  QrCorrection correction;
  SYSTIM correctionUntil;
  pup_device_t* colorSensor;
  pup_motor_t* leftMotor;
  pup_motor_t* rightMotor;
  float integral;
  float previousError;
  SYSTIM now;

  get_tim(&now);

  loc_mtx(CONTROL_MUTEX);
  running = state_.running;
  config = state_.config;
  correction = state_.correction;
  correctionUntil = state_.correctionUntil;
  colorSensor = state_.colorSensor;
  leftMotor = state_.leftMotor;
  rightMotor = state_.rightMotor;
  integral = state_.integral;
  previousError = state_.previousError;
  qrActive = state_.status.qr_active && now <= correctionUntil;
  if (!qrActive) {
    state_.status.qr_active = 0;
  }
  unl_mtx(CONTROL_MUTEX);

  if (!running || colorSensor == nullptr || leftMotor == nullptr || rightMotor == nullptr) {
    return;
  }

  int32_t basePower = config.base_power;
  int32_t targetReflection = config.target_reflection;
  int32_t turnBias = 0;
  if (qrActive) {
    basePower += correction.base_power_delta;
    targetReflection += correction.target_reflection_delta;
    turnBias = correction.turn_bias;
  }

  basePower = clampI32(basePower, -config.max_power, config.max_power);
  targetReflection = clampI32(targetReflection, 0, 100);

  int32_t reflection = pup_color_sensor_reflection(colorSensor);
  float error = static_cast<float>(targetReflection - reflection);
  integral = clampFloat(integral + error, -config.integral_limit, config.integral_limit);
  float derivative = error - previousError;
  int32_t turn = static_cast<int32_t>(config.kp * error + config.ki * integral +
                                      config.kd * derivative) + turnBias;
  turn = clampI32(turn, -config.turn_limit, config.turn_limit);

  int32_t leftPower = clampI32(basePower + turn * config.edge, -config.max_power,
                               config.max_power);
  int32_t rightPower = clampI32(basePower - turn * config.edge, -config.max_power,
                                config.max_power);

  pbio_error_t leftErr = pup_motor_set_power(leftMotor, leftPower);
  pbio_error_t rightErr = pup_motor_set_power(rightMotor, rightPower);

  loc_mtx(CONTROL_MUTEX);
  state_.integral = integral;
  state_.previousError = error;
  state_.status.configured = state_.configured ? 1 : 0;
  state_.status.running = state_.running ? 1 : 0;
  state_.status.reflection = reflection;
  state_.status.error = static_cast<int32_t>(error);
  state_.status.turn = turn;
  state_.status.left_power = leftPower;
  state_.status.right_power = rightPower;
  state_.status.loop_count++;
  if (leftErr != PBIO_SUCCESS || rightErr != PBIO_SUCCESS) {
    state_.status.fault_count++;
  }
  unl_mtx(CONTROL_MUTEX);
}

int32_t LineController::clampI32(int32_t value, int32_t minValue, int32_t maxValue)
{
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float LineController::clampFloat(float value, float minValue, float maxValue)
{
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

bool LineController::validPort(uint8_t port)
{
  return port < kMaxDevices;
}

}  // namespace control
}  // namespace etrobo
