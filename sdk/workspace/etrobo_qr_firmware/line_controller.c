#include "line_controller.h"

#include "app.h"
#include "kernel_cfg.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <pbio/error.h>
#include <spike/pup/colorsensor.h>
#include <spike/pup/motor.h>

#define APP_MAX_DEVICES       (6)
#define APP_MAX_POWER         (100)
#define APP_MAX_QR_VALID_US   (5 * 1000 * 1000)

#define PORT_FROM_INDEX(port) ((port) + 'A')

typedef struct {
  bool configured;
  bool running;
  LineConfig config;
  QrCorrection correction;
  SYSTIM correction_until;
  uint32_t last_qr_seq;
  float integral;
  float previous_error;
  pup_device_t *color_sensor;
  pup_motor_t *left_motor;
  pup_motor_t *right_motor;
  LineStatus status;
} ControlState;

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static bool valid_port(uint8_t port)
{
  return port < APP_MAX_DEVICES;
}

static const LineConfig DEFAULT_CONFIG = {
  .color_port = 4,
  .left_motor_port = 1,
  .right_motor_port = 0,
  .edge = 1,
  .left_direction = PUP_DIRECTION_COUNTERCLOCKWISE,
  .right_direction = PUP_DIRECTION_CLOCKWISE,
  .reset_count = 1,
  .target_reflection = 45,
  .base_power = 30,
  .max_power = 60,
  .turn_limit = 50,
  .kp = 0.4f,
  .ki = 0.0f,
  .kd = 0.0f,
  .integral_limit = 1000.0f,
};

static ControlState g_control;

void line_controller_configure_default(void)
{
  line_controller_configure(&DEFAULT_CONFIG);
}

int line_controller_configure(const LineConfig *config)
{
  if (!config ||
      !valid_port(config->color_port) ||
      !valid_port(config->left_motor_port) ||
      !valid_port(config->right_motor_port)) {
    return 0;
  }

  pup_device_t *color_sensor = pup_color_sensor_get_device(PORT_FROM_INDEX(config->color_port));
  pup_motor_t *left_motor = pup_motor_get_device(PORT_FROM_INDEX(config->left_motor_port));
  pup_motor_t *right_motor = pup_motor_get_device(PORT_FROM_INDEX(config->right_motor_port));
  if (!color_sensor || !left_motor || !right_motor) {
    return 0;
  }

  pup_direction_t left_direction =
      (config->left_direction == PUP_DIRECTION_COUNTERCLOCKWISE) ?
      PUP_DIRECTION_COUNTERCLOCKWISE : PUP_DIRECTION_CLOCKWISE;
  pup_direction_t right_direction =
      (config->right_direction == PUP_DIRECTION_COUNTERCLOCKWISE) ?
      PUP_DIRECTION_COUNTERCLOCKWISE : PUP_DIRECTION_CLOCKWISE;
  bool reset_count = config->reset_count ? true : false;
  if (pup_motor_setup(left_motor, left_direction, reset_count) != PBIO_SUCCESS ||
      pup_motor_setup(right_motor, right_direction, reset_count) != PBIO_SUCCESS) {
    return 0;
  }

  LineConfig normalized = *config;
  normalized.edge = (normalized.edge < 0) ? -1 : 1;
  normalized.max_power = clamp_i32(normalized.max_power, 1, APP_MAX_POWER);
  normalized.base_power = clamp_i32(normalized.base_power, -normalized.max_power, normalized.max_power);
  normalized.turn_limit = clamp_i32(normalized.turn_limit, 1, normalized.max_power);
  if (normalized.integral_limit <= 0.0f) {
    normalized.integral_limit = 1000.0f;
  }

  loc_mtx(CONTROL_MUTEX);
  g_control.config = normalized;
  g_control.color_sensor = color_sensor;
  g_control.left_motor = left_motor;
  g_control.right_motor = right_motor;
  g_control.configured = true;
  g_control.running = false;
  g_control.integral = 0.0f;
  g_control.previous_error = 0.0f;
  memset(&g_control.correction, 0, sizeof(g_control.correction));
  memset(&g_control.status, 0, sizeof(g_control.status));
  g_control.status.configured = 1;
  unl_mtx(CONTROL_MUTEX);

  return 1;
}

void line_controller_stop(void)
{
  pup_motor_t *left_motor;
  pup_motor_t *right_motor;

  loc_mtx(CONTROL_MUTEX);
  g_control.running = false;
  g_control.status.running = 0;
  left_motor = g_control.left_motor;
  right_motor = g_control.right_motor;
  unl_mtx(CONTROL_MUTEX);

  stp_cyc(LINE_TRACER_CYC);
  if (left_motor) {
    pup_motor_stop(left_motor);
  }
  if (right_motor) {
    pup_motor_stop(right_motor);
  }
}

int line_controller_start(void)
{
  int ok;

  loc_mtx(CONTROL_MUTEX);
  ok = g_control.configured ? 1 : 0;
  if (ok) {
    g_control.running = true;
    g_control.integral = 0.0f;
    g_control.previous_error = 0.0f;
    g_control.status.running = 1;
  }
  unl_mtx(CONTROL_MUTEX);

  if (ok) {
    sta_cyc(LINE_TRACER_CYC);
  }

  return ok;
}

int line_controller_apply_qr(const QrCorrection *correction)
{
  SYSTIM now;

  if (!correction) {
    return 0;
  }
  get_tim(&now);

  loc_mtx(CONTROL_MUTEX);
  if (correction->seq <= g_control.last_qr_seq) {
    unl_mtx(CONTROL_MUTEX);
    return 0;
  }

  g_control.correction = *correction;
  g_control.correction.base_power_delta = clamp_i32(correction->base_power_delta, -30, 30);
  g_control.correction.target_reflection_delta = clamp_i32(correction->target_reflection_delta, -30, 30);
  g_control.correction.turn_bias = clamp_i32(correction->turn_bias, -30, 30);
  uint32_t valid_us = correction->valid_ms * 1000;
  if (valid_us == 0 || valid_us > APP_MAX_QR_VALID_US) {
    valid_us = APP_MAX_QR_VALID_US;
  }
  g_control.correction_until = now + valid_us;
  g_control.last_qr_seq = correction->seq;
  g_control.status.qr_seq = correction->seq;
  g_control.status.qr_id = correction->qr_id;
  g_control.status.mode = correction->mode;
  g_control.status.qr_active = 1;
  unl_mtx(CONTROL_MUTEX);

  return 1;
}

void line_controller_get_status(LineStatus *status)
{
  if (!status) {
    return;
  }

  loc_mtx(CONTROL_MUTEX);
  *status = g_control.status;
  status->configured = g_control.configured ? 1 : 0;
  status->running = g_control.running ? 1 : 0;
  unl_mtx(CONTROL_MUTEX);
}

void line_controller_run_cycle(void)
{
  bool running;
  bool qr_active;
  LineConfig config;
  QrCorrection correction;
  SYSTIM correction_until;
  pup_device_t *color_sensor;
  pup_motor_t *left_motor;
  pup_motor_t *right_motor;
  float integral;
  float previous_error;
  SYSTIM now;

  get_tim(&now);

  loc_mtx(CONTROL_MUTEX);
  running = g_control.running;
  config = g_control.config;
  correction = g_control.correction;
  correction_until = g_control.correction_until;
  color_sensor = g_control.color_sensor;
  left_motor = g_control.left_motor;
  right_motor = g_control.right_motor;
  integral = g_control.integral;
  previous_error = g_control.previous_error;
  qr_active = g_control.status.qr_active && now <= correction_until;
  if (!qr_active) {
    g_control.status.qr_active = 0;
  }
  unl_mtx(CONTROL_MUTEX);

  if (!running || !color_sensor || !left_motor || !right_motor) {
    return;
  }

  int32_t base_power = config.base_power;
  int32_t target_reflection = config.target_reflection;
  int32_t turn_bias = 0;
  if (qr_active) {
    base_power += correction.base_power_delta;
    target_reflection += correction.target_reflection_delta;
    turn_bias = correction.turn_bias;
  }

  base_power = clamp_i32(base_power, -config.max_power, config.max_power);
  target_reflection = clamp_i32(target_reflection, 0, 100);

  int32_t reflection = pup_color_sensor_reflection(color_sensor);
  float error = (float)(target_reflection - reflection);
  integral = clamp_float(integral + error, -config.integral_limit, config.integral_limit);
  float derivative = error - previous_error;
  int32_t turn = (int32_t)(config.kp * error + config.ki * integral + config.kd * derivative) + turn_bias;
  turn = clamp_i32(turn, -config.turn_limit, config.turn_limit);

  int32_t left_power = clamp_i32(base_power + turn * config.edge, -config.max_power, config.max_power);
  int32_t right_power = clamp_i32(base_power - turn * config.edge, -config.max_power, config.max_power);

  pbio_error_t left_err = pup_motor_set_power(left_motor, left_power);
  pbio_error_t right_err = pup_motor_set_power(right_motor, right_power);

  loc_mtx(CONTROL_MUTEX);
  g_control.integral = integral;
  g_control.previous_error = error;
  g_control.status.configured = g_control.configured ? 1 : 0;
  g_control.status.running = g_control.running ? 1 : 0;
  g_control.status.reflection = reflection;
  g_control.status.error = (int32_t)error;
  g_control.status.turn = turn;
  g_control.status.left_power = left_power;
  g_control.status.right_power = right_power;
  g_control.status.loop_count++;
  if (left_err != PBIO_SUCCESS || right_err != PBIO_SUCCESS) {
    g_control.status.fault_count++;
  }
  unl_mtx(CONTROL_MUTEX);
}
