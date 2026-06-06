#ifndef ETROBO_QR_FIRMWARE_PROTOCOL_H
#define ETROBO_QR_FIRMWARE_PROTOCOL_H

#include <stdint.h>

#define APP_FRAME_START (0xEA)

#define APP_CMD_CONFIG        (0x01)
#define APP_CMD_START         (0x02)
#define APP_CMD_STOP          (0x03)
#define APP_CMD_QR_CORRECTION (0x04)
#define APP_CMD_STATUS        (0x05)

#define APP_MSG_ACK           (0x80)
#define APP_MSG_STATUS        (0x81)

typedef struct {
  uint8_t color_port;
  uint8_t left_motor_port;
  uint8_t right_motor_port;
  int8_t edge;
  int8_t left_direction;
  int8_t right_direction;
  uint8_t reset_count;
  uint8_t reserved;
  int32_t target_reflection;
  int32_t base_power;
  int32_t max_power;
  int32_t turn_limit;
  float kp;
  float ki;
  float kd;
  float integral_limit;
} LineConfig;

typedef struct {
  uint32_t qr_id;
  uint32_t seq;
  int32_t base_power_delta;
  int32_t target_reflection_delta;
  int32_t turn_bias;
  uint32_t valid_ms;
  uint8_t mode;
  uint8_t reserved[3];
} QrCorrection;

typedef struct {
  uint8_t configured;
  uint8_t running;
  uint8_t qr_active;
  uint8_t mode;
  int32_t reflection;
  int32_t error;
  int32_t turn;
  int32_t left_power;
  int32_t right_power;
  uint32_t loop_count;
  uint32_t fault_count;
  uint32_t qr_seq;
  uint32_t qr_id;
} LineStatus;

#endif
