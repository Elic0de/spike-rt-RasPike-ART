/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 Yuki Tsuhitoi
 */

#include <kernel.h>
#include <kernel_cfg.h>
#include <t_syslog.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include <syssvc/serial.h>
#include <serial/serial.h>
#include <spike/hub/system.h>
#include <spike/hub/battery.h>
#include <spike/hub/button.h>
#include <spike/hub/display.h>
#include <spike/hub/imu.h>
#include <spike/hub/light.h>
#include <spike/hub/speaker.h>
#include <pbdrv/reset.h>
#include <pbdrv/usb.h>
#include <pbdrv/watchdog.h>
#include <pbsys/status.h>

#include <spike/pup/motor.h>
#include <spike/pup/colorsensor.h>
#include <spike/pup/forcesensor.h>
#include <spike/pup/ultrasonicsensor.h>


#include "raspike.h"
#define RP_DEFINE_CMD_SIZE
#include "raspike_protocol_com.h"
#include "hub_fault_injection.h"
#include "hub_runtime_state.h"
#include "line_trace_controller.h"
#include "line_trace_speed_controller.h"
//#include "tPutLogTarget_tecsgen.h"

_Static_assert(RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD == 256u,
               "RasPike v2 frame must fit in one 256-byte buffer");
_Static_assert(sizeof(RPProtocolSpikeStatus) <= RP_V2_MAX_PAYLOAD,
               "status payload exceeds protocol v2 maximum");
_Static_assert(sizeof(RPDriveTelemetry) <= RP_V2_MAX_PAYLOAD,
               "drive telemetry exceeds protocol v2 maximum");
_Static_assert(sizeof(RPRuntimeTelemetry) <= RP_V2_MAX_PAYLOAD,
               "runtime telemetry exceeds protocol v2 maximum");

#define RP_MAX_DEVICES (6)

#ifndef SPIKE_EXPECTED_VERSION_MAJOR
#define SPIKE_EXPECTED_VERSION_MAJOR 0
#endif
#ifndef SPIKE_EXPECTED_VERSION_MINOR
#define SPIKE_EXPECTED_VERSION_MINOR 0
#endif
#ifndef SPIKE_EXPECTED_VERSION_PATCH
#define SPIKE_EXPECTED_VERSION_PATCH 0
#endif

#define RP_LINK_RENEGOTIATED (-1000)
#define RP_REQUEST_REPLAYED (-1001)
#define RP_CRITICAL_TX_SLOT_COUNT 8u
#define RP_CRITICAL_TX_FRAME_CAPACITY 260u
#define RP_MAINTENANCE_TX_TIMEOUT_US 500000u
#define RP_SAFETY_INHIBIT_USB       (1u << 0)
#define RP_SAFETY_INHIBIT_BATTERY   (1u << 1)
#define RP_SAFETY_INHIBIT_CONTROL   (1u << 2)
#define RP_SAFETY_CONTROL_STALE_CYCLES 6u

static RPProtocolSpikeStatus fgCurrentStatus = {0};

typedef struct {
  uint32_t rx_frames;
  uint32_t tx_frames;
  uint32_t crc_errors;
  uint32_t framing_errors;
  uint32_t session_errors;
  uint32_t invalid_commands;
  uint32_t duplicate_configs;
  uint32_t duplicate_requests;
} RPLinkStats;

typedef struct {
  uint32_t session;
  uint32_t sequence;
  unsigned char command;
  uint16_t request_crc;
  int32_t result;
  bool valid;
} RPCachedAck;

typedef struct {
  void *device;
  unsigned char config;
  unsigned char next_cmd;
} RPDevice;

typedef enum {
  RP_TX_SLOT_FREE = 0,
  RP_TX_SLOT_WRITING,
  RP_TX_SLOT_READY,
  RP_TX_SLOT_SENDING,
} RPTxSlotState;

typedef struct {
  uint16_t size;
  uint32_t ticket;
  volatile RPTxSlotState state;
  unsigned char data[RP_CRITICAL_TX_FRAME_CAPACITY];
} RPCriticalTxSlot;

static RPDevice fgDevices[RP_MAX_DEVICES] = {0};
static uint32_t fgSessionId = 0;
static uint32_t fgTxSequence = 1;
static uint32_t fgActiveRequestSequence = 0;
static uint32_t fgActiveRequestSession = 0;
static unsigned char fgActiveRequestFlags = 0;
static uint16_t fgActiveRequestCrc = 0;
static bool fgProtocolV2Active = false;
static RPLinkStats fgLinkStats = {0};
static RPCachedAck fgAckCache[RP_MAX_DEVICES + 1] = {0};
static RPCriticalTxSlot fgCriticalTxSlots[RP_CRITICAL_TX_SLOT_COUNT] = {0};
static volatile uint8_t fgCriticalTxHead = 0u;
static volatile uint8_t fgCriticalTxTail = 0u;
static volatile uint8_t fgCriticalTxCount = 0u;
static volatile bool fgCriticalTxTaskScheduled = false;
static volatile uint32_t fgCriticalTxNextTicket = 1u;
static volatile uint32_t fgCriticalTxCompletedTicket = 0u;
static volatile bool fgSafetyTaskPending = false;
static uint32_t fgSafetyUsbTxRecoverySeen = 0u;
static uint32_t fgSafetyUsbReinitSeen = 0u;
static uint32_t fgSafetyUsbRxOverflowSeen = 0u;
static volatile uint32_t fgSafetyInhibitReasons = 0u;
static volatile uint32_t fgSafetyGeneration = 0u;
static uint32_t fgSafetyLastControlUpdateCount = 0u;
static uint8_t fgSafetyControlStaleCycles = 0u;
static bool fgSafetyUsbWasConnected = false;
static void update_port_config(RasPikePort port, const int type, void *device);
static void update_device_cmd(RasPikePort port, const int cmd_id, char *data, size_t data_size);
static void reset_runtime_state_for_restart(void);
static void stop_all_motors_for_reset(void);
static int begin_link_session(bool reset_runtime);
static int replay_cached_ack(RasPikePort port, unsigned char command,
                             uint32_t sequence, uint32_t session,
                             uint16_t request_crc);
static int enqueue_critical_tx_with_ticket(const unsigned char *frame,
                                           size_t size,
                                           uint32_t *ticket_out);
static int enqueue_critical_tx(const unsigned char *frame, size_t size);
static bool wait_for_critical_tx(uint32_t ticket, uint32_t timeout_us);
static bool hub_maintenance_transition(RasPikePort port, int command_id,
                                       pbdrv_reset_action_t action);

void target_emergency_stop(void)
{
  hub_motor_emergency_disable_from_isr();
}

#define RP_DRIVE_RIGHT_MOTOR_PORT 0
#define RP_DRIVE_LEFT_MOTOR_PORT  1
#define RP_DRIVE_COLOR_PORT       4
#define RP_DRIVE_WATCHDOG_MS      300
#define RP_DRIVE_PWM_LIMIT        100
#define RP_DRIVE_YAW_MAX_PWM      75.0f
#define RP_DRIVE_YAW_RATE_SCALE   0.99479512f
#define RP_DRIVE_DEG_TO_RAD       0.01745329251994329577f
#define RP_DRIVE_YAW_MAX_RAD_S    0.2f
#define RP_DRIVE_YAW_GYRO_KP      2.0f
#define RP_DRIVE_POSITIVE_TURN_MM_PER_RAD 113.0f
#define RP_DRIVE_NEGATIVE_TURN_MM_PER_RAD 94.0f

typedef struct {
  RPRealtimeDriveCommand command;
  LineTraceSpeedControllerParameters controller_parameters;
  uint32_t received_time_ms;
  float target_speed_mm_s;
  float curvature_per_mm;
  float gyro_bias_deg_s;
  float dt_s;
} RPPreparedDriveCommand;

static pup_motor_t *fgDriveLeftMotor = NULL;
static pup_motor_t *fgDriveRightMotor = NULL;
static pup_device_t *fgDriveColorSensor = NULL;
static RPPreparedDriveCommand fgDriveCommandSlots[2] = {0};
static volatile uint8_t fgDriveCommandPublishedSlot = 0u;
static uint32_t fgDrivePreviousControlUs = 0;
static int32_t fgDriveIntegral = 0;
static int16_t fgDrivePreviousError = 0;
static uint16_t fgDriveStatusFlags = 0;
static bool fgDriveConfigured = false;
static bool fgDriveRunning = false;
static bool fgDriveMotorsStopped = true;
static uint16_t fgDriveControlPeriodUs = 4000u;
static LineTraceSpeedControllerState fgDriveLineControllerState = {0};
static LineTraceSpeedControllerOutput fgDriveLineControllerOutput = {0};
static uint32_t fgDriveControllerUpdateCount = 0;
static uint32_t fgDriveControllerDeadlineMissCount = 0;
static uint64_t fgDriveLineErrorSqSum = 0;
static uint32_t fgDriveLineErrorAbsSum = 0;
static uint16_t fgDriveLineErrorAbsMax = 0;
static uint16_t fgDrivePwmSaturationSamples = 0;
static uint32_t fgDriveControllerPeriodUs = 0;
static uint32_t fgDriveControllerExecutionUs = 0;
static uint32_t fgDriveUsbTxRecoverySeen = 0;
static uint32_t fgDriveUsbReinitSeen = 0;
static uint32_t fgDriveUsbRxOverflowSeen = 0;
static volatile bool fgDriveControlTaskPending = false;
static uint32_t fgDriveTelemetrySequence = 0;
static const HubRuntimeParameters fgRuntimeParameters = {
  .degraded_limit_cycles = 5u,
  .recovery_healthy_cycles = 2u,
};
static HubRuntimeState fgRuntimeState = {0};
static HubFaultInjectionState fgRuntimeFault = {0};
static RPRuntimeTelemetry fgRuntimeTelemetrySnapshot = {0};
static uint32_t fgRuntimeTelemetrySequence = 0;
static uint32_t fgRuntimeSampleSequence = 0;
static uint32_t fgRuntimeOverrunCount = 0;
static bool fgRuntimeRecoverRequested = false;
static bool fgRuntimeImuReadyLatched = false;
static pup_motor_t *fgDriveSetupLeftMotor = NULL;
static pup_motor_t *fgDriveSetupRightMotor = NULL;

static uint32_t realtime_drive_now_us(void)
{
  SYSTIM now = 0;
  get_tim(&now);
  return (uint32_t)now;
}

static uint32_t realtime_drive_now_ms(void)
{
  return realtime_drive_now_us() / 1000u;
}

static uint32_t fgDriveCpuCyclesPerUs = 1u;

static void realtime_drive_cycle_counter_init(void)
{
  const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
  fgDriveCpuCyclesPerUs = cycles_per_us > 0u ? cycles_per_us : 1u;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
}

static uint32_t realtime_drive_now_cycles(void)
{
  return DWT->CYCCNT;
}

static uint32_t realtime_drive_cycles_to_us(uint32_t cycles)
{
  return (cycles + fgDriveCpuCyclesPerUs - 1u)
      / fgDriveCpuCyclesPerUs;
}

static uint16_t realtime_drive_normalize_control_period(uint16_t requested_us);

static RPPreparedDriveCommand realtime_drive_prepare_command(
    const RPRealtimeDriveCommand *command, uint32_t received_time_ms)
{
  RPPreparedDriveCommand prepared = {0};
  prepared.command = *command;
  prepared.received_time_ms = received_time_ms;
  prepared.target_speed_mm_s = (float)command->target_speed_mm_s;
  prepared.curvature_per_mm = (float)command->curvature_per_mm_e9 * 1.0e-9f;
  prepared.gyro_bias_deg_s = (float)command->left_pwm * 0.001f;
  prepared.dt_s = realtime_drive_normalize_control_period(
      command->control_period_us) == 10000u ? 0.010f : 0.004f;

  prepared.controller_parameters =
      line_trace_speed_controller_default_parameters();
  prepared.controller_parameters.positive_turn_mm_per_robot_rad =
      RP_DRIVE_POSITIVE_TURN_MM_PER_RAD;
  prepared.controller_parameters.negative_turn_mm_per_robot_rad =
      RP_DRIVE_NEGATIVE_TURN_MM_PER_RAD;
  prepared.controller_parameters.max_omega_rad_s = RP_DRIVE_YAW_MAX_RAD_S;

  if (command->mode == RP_DRIVE_MODE_YAW_RATE) {
    prepared.controller_parameters.line.target_reflection = 0;
    prepared.controller_parameters.line.kp_milli = 0;
    prepared.controller_parameters.line.ki_milli = 0;
    prepared.controller_parameters.line.kd_milli = 0;
    prepared.controller_parameters.gyro_kp =
        (float)command->kp_milli * 0.001f;
    prepared.controller_parameters.max_pwm = RP_DRIVE_YAW_MAX_PWM;
  } else {
    prepared.controller_parameters.line.target_reflection =
        command->target_reflection;
    prepared.controller_parameters.line.kp_milli = command->kp_milli;
    prepared.controller_parameters.line.ki_milli = command->ki_milli;
    prepared.controller_parameters.line.kd_milli = command->kd_milli;
    prepared.controller_parameters.line.pwm_headroom_margin =
        (int16_t)(command->reserved0 & 0xffu);
    prepared.controller_parameters.wheel_speed_pi_enabled =
        (command->reserved0 & 0x0100u) != 0u;
    prepared.controller_parameters.gyro_kp = RP_DRIVE_YAW_GYRO_KP;
  }

  return prepared;
}

static void realtime_drive_publish_command(
    const RPRealtimeDriveCommand *command, uint32_t received_time_ms)
{
  const uint8_t next_slot = (uint8_t)(fgDriveCommandPublishedSlot ^ 1u);
  fgDriveCommandSlots[next_slot] =
      realtime_drive_prepare_command(command, received_time_ms);

  /* The command task is lower priority than the control task. It writes only
     the unpublished slot, then performs a release-style atomic publication. */
  __DMB();
  loc_cpu();
  fgDriveCommandPublishedSlot = next_slot;
  unl_cpu();
}

static RPPreparedDriveCommand realtime_drive_snapshot_command(void)
{
  const uint8_t slot = fgDriveCommandPublishedSlot;
  __DMB();
  return fgDriveCommandSlots[slot];
}

static void realtime_drive_reset_command_mailbox(void)
{
  memset(fgDriveCommandSlots, 0, sizeof(fgDriveCommandSlots));
  fgDriveCommandPublishedSlot = 0u;
}

static int realtime_drive_clamp(int value, int minimum, int maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static uint16_t realtime_drive_normalize_control_period(uint16_t requested_us)
{
  return requested_us >= 7000u ? 10000u : 4000u;
}

static void realtime_drive_select_control_period(uint16_t requested_us)
{
  const uint16_t normalized = realtime_drive_normalize_control_period(requested_us);
  if (fgDriveControlPeriodUs == normalized && fgDrivePreviousControlUs != 0u) {
    return;
  }
  stp_cyc(APP_DRIVE_CONTROL_4MS_CYC);
  stp_cyc(APP_DRIVE_CONTROL_10MS_CYC);
  fgDriveControlPeriodUs = normalized;
  fgDrivePreviousControlUs = 0u;
  if (normalized == 10000u) {
    sta_cyc(APP_DRIVE_CONTROL_10MS_CYC);
  } else {
    sta_cyc(APP_DRIVE_CONTROL_4MS_CYC);
  }
}

static void realtime_drive_reset_controller_metrics(void)
{
  line_trace_speed_controller_reset(&fgDriveLineControllerState);
  fgDriveLineControllerOutput = (LineTraceSpeedControllerOutput){0};
  fgDriveControllerUpdateCount = 0u;
  fgDriveControllerDeadlineMissCount = 0u;
  fgDriveLineErrorSqSum = 0u;
  fgDriveLineErrorAbsSum = 0u;
  fgDriveLineErrorAbsMax = 0u;
  fgDrivePwmSaturationSamples = 0u;
  fgDriveControllerPeriodUs = 0u;
  fgDriveControllerExecutionUs = 0u;
}

static void realtime_drive_stop(bool watchdog)
{
  fgDriveRunning = false;
  fgDriveStatusFlags &= ~RP_DRIVE_STATUS_RUNNING;
  if (watchdog) fgDriveStatusFlags |= RP_DRIVE_STATUS_WATCHDOG_STOP;
}

static int32_t realtime_drive_configure_devices(void)
{
  if (fgDriveConfigured) return 1;

  fgDriveRightMotor = pup_motor_get_device(PORT_FROM_RASPIKE(RP_DRIVE_RIGHT_MOTOR_PORT));
  fgDriveLeftMotor = pup_motor_get_device(PORT_FROM_RASPIKE(RP_DRIVE_LEFT_MOTOR_PORT));
  fgDriveColorSensor = pup_color_sensor_get_device(PORT_FROM_RASPIKE(RP_DRIVE_COLOR_PORT));
  if (!fgDriveRightMotor || !fgDriveLeftMotor || !fgDriveColorSensor) {
    fgDriveStatusFlags |= RP_DRIVE_STATUS_SENSOR_ERROR;
    return 0;
  }

  if (fgDriveSetupLeftMotor != fgDriveLeftMotor) {
    if (pup_motor_setup(fgDriveLeftMotor, PUP_DIRECTION_COUNTERCLOCKWISE, true) != PBIO_SUCCESS) return 0;
    fgDriveSetupLeftMotor = fgDriveLeftMotor;
  }
  if (fgDriveSetupRightMotor != fgDriveRightMotor) {
    if (pup_motor_setup(fgDriveRightMotor, PUP_DIRECTION_CLOCKWISE, true) != PBIO_SUCCESS) return 0;
    fgDriveSetupRightMotor = fgDriveRightMotor;
  }
  /* End any Pybricks parent control once. The 4 ms loop then uses the
     non-parent-stopping real-time power path. */
  if (pup_motor_brake(fgDriveLeftMotor) != PBIO_SUCCESS
      || pup_motor_brake(fgDriveRightMotor) != PBIO_SUCCESS) {
    return 0;
  }

  update_port_config(RP_DRIVE_RIGHT_MOTOR_PORT, RP_CMD_TYPE_MOTOR, fgDriveRightMotor);
  update_port_config(RP_DRIVE_LEFT_MOTOR_PORT, RP_CMD_TYPE_MOTOR, fgDriveLeftMotor);
  update_port_config(RP_DRIVE_COLOR_PORT, RP_CMD_TYPE_COLOR, fgDriveColorSensor);
  update_device_cmd(RP_DRIVE_RIGHT_MOTOR_PORT, RP_CMD_ID_MOT_STU, 0, 0);
  update_device_cmd(RP_DRIVE_LEFT_MOTOR_PORT, RP_CMD_ID_MOT_STU, 0, 0);
  update_device_cmd(RP_DRIVE_COLOR_PORT, RP_CMD_ID_COL_REF, 0, 0);

  fgDriveConfigured = true;
  fgDriveStatusFlags = RP_DRIVE_STATUS_ARMED;
  fgDriveMotorsStopped = true;
  hub_runtime_reset(&fgRuntimeState);
  hub_fault_injection_clear(&fgRuntimeFault);
  memset(&fgRuntimeTelemetrySnapshot, 0, sizeof(fgRuntimeTelemetrySnapshot));
  fgRuntimeTelemetrySequence = 0;
  fgRuntimeSampleSequence = 0;
  fgRuntimeOverrunCount = 0;
  fgRuntimeRecoverRequested = true;
  fgDriveUsbTxRecoverySeen = pbdrv_usb_get_tx_recovery_count();
  fgDriveUsbReinitSeen = pbdrv_usb_get_reinit_count();
  fgDriveUsbRxOverflowSeen = pbdrv_usb_get_rx_overflow_count();
  fgDriveControlTaskPending = false;
  realtime_drive_reset_controller_metrics();
  /* AB mode exclusively owns these devices. Disable the legacy notifier so
     it cannot contend with the 4 ms loop or add unrelated USB traffic. */
  stp_cyc(APP_NOTIFY_CYC);
  stp_cyc(APP_DRIVE_TELEMETRY_CYC);
  fgDrivePreviousControlUs = 0u;
  realtime_drive_select_control_period(fgDriveControlPeriodUs);
  sta_cyc(APP_RUNTIME_TELEMETRY_CYC);
  return 1;
}

static void lock_status(void)
{
  loc_mtx(APP_STATUS_MUTEX);
}

static void unlock_status(void)
{
  unl_mtx(APP_STATUS_MUTEX);
}

static int32_t read_i32(const char *data)
{
  int32_t value = 0;
  memcpy(&value, data, sizeof(value));
  return value;
}

static uint32_t read_u32(const char *data)
{
  uint32_t value = 0;
  memcpy(&value, data, sizeof(value));
  return value;
}

static uint16_t read_u16(const char *data)
{
  uint16_t value = 0;
  memcpy(&value, data, sizeof(value));
  return value;
}

static float read_float32(const char *data)
{
  float value = 0.0f;
  memcpy(&value, data, sizeof(value));
  return value;
}

static bool read_bool_value(const char *data)
{
  bool value = false;
  memcpy(&value, data, sizeof(value));
  return value;
}


static int is_valid_protocol_port(RasPikePort port)
{
  return port < RP_MAX_DEVICES || port == RP_PORT_NONE;
}

static uint32_t next_tx_sequence(void)
{
  loc_mtx(APP_SEQUENCE_MUTEX);
  uint32_t sequence = fgTxSequence++;
  if (fgTxSequence == 0) fgTxSequence = 1;
  unl_mtx(APP_SEQUENCE_MUTEX);
  return sequence;
}

static int send_data_blocking(const char *buf, size_t size)
{
  if (!buf && size) return E_PAR;
  size_t sent = 0;
  while (sent < size) {
    ER_UINT result = serial_wri_dat(SIO_USB_PORTID, buf + sent, size - sent);
    if (result < 0) return (int)result;
    if (result == 0) {
      dly_tsk(1000);
      continue;
    }
    sent += (size_t)result;
  }
  return (int)sent;
}

static int enqueue_critical_tx_with_ticket(const unsigned char *frame,
                                           size_t size,
                                           uint32_t *ticket_out)
{
  if ((!frame && size > 0u) || size > RP_CRITICAL_TX_FRAME_CAPACITY) {
    return E_PAR;
  }

  RPCriticalTxSlot *slot;
  bool wake_task = false;
  loc_cpu();
  if (fgCriticalTxCount >= RP_CRITICAL_TX_SLOT_COUNT) {
    unl_cpu();
    return E_QOVR;
  }
  slot = &fgCriticalTxSlots[fgCriticalTxTail];
  if (slot->state != RP_TX_SLOT_FREE) {
    unl_cpu();
    return E_SYS;
  }
  slot->state = RP_TX_SLOT_WRITING;
  slot->ticket = fgCriticalTxNextTicket++;
  if (fgCriticalTxNextTicket == 0u) fgCriticalTxNextTicket = 1u;
  if (ticket_out) *ticket_out = slot->ticket;
  fgCriticalTxTail = (uint8_t)((fgCriticalTxTail + 1u) % RP_CRITICAL_TX_SLOT_COUNT);
  ++fgCriticalTxCount;
  unl_cpu();

  if (size > 0u) memcpy(slot->data, frame, size);
  slot->size = (uint16_t)size;

  loc_cpu();
  slot->state = RP_TX_SLOT_READY;
  if (!fgCriticalTxTaskScheduled) {
    fgCriticalTxTaskScheduled = true;
    wake_task = true;
  }
  unl_cpu();
  if (wake_task && act_tsk(APP_CRITICAL_TX_TASK) < 0) {
    loc_cpu();
    fgCriticalTxTaskScheduled = false;
    unl_cpu();
    return E_SYS;
  }
  return (int)size;
}

static int enqueue_critical_tx(const unsigned char *frame, size_t size)
{
  return enqueue_critical_tx_with_ticket(frame, size, NULL);
}

static bool wait_for_critical_tx(uint32_t ticket, uint32_t timeout_us)
{
  const uint32_t started_us = realtime_drive_now_us();
  while ((uint32_t)(realtime_drive_now_us() - started_us) < timeout_us) {
    T_SERIAL_RPOR state = {0};
    const bool ticket_done = fgCriticalTxCompletedTicket >= ticket;
    const bool critical_empty = fgCriticalTxCount == 0u;
    const bool serial_empty = serial_ref_por(SIO_USB_PORTID, &state) >= 0
        && state.wricnt == 0u;
    if (ticket_done && critical_empty && serial_empty
        && pbdrv_usb_is_tx_idle()) {
      return true;
    }
    dly_tsk(1000);
  }
  return false;
}

static int wait_read(char *buf, size_t size)
{
  if (!buf && size) return E_PAR;
  size_t received = 0;
  while (received < size) {
    ER_UINT result = serial_rea_dat(SIO_USB_PORTID, buf + received, size - received);
    if (result < 0) return (int)result;
    if (result == 0) {
      dly_tsk(1000);
      continue;
    }
    received += (size_t)result;
  }
  return (int)received;
}

static int discard_bytes(size_t size)
{
  char scratch[32];
  while (size > 0) {
    size_t chunk = size < sizeof(scratch) ? size : sizeof(scratch);
    int result = wait_read(scratch, chunk);
    if (result < 0) return result;
    size -= chunk;
  }
  return 0;
}

static int raspike_send_legacy(RasPikePort port, int msg_id,
                               const char *buf, size_t size)
{
  if (!is_valid_protocol_port(port) || size > 255u
      || (size > 0 && !buf)) return E_PAR;
  unsigned char frame[4u + 255u] = {
    RP_CMD_START, (unsigned char)msg_id, (unsigned char)size, port
  };
  if (buf && size > 0u) memcpy(frame + 4u, buf, size);
  const int result = enqueue_critical_tx(frame, 4u + size);
  if (result >= 0) ++fgLinkStats.tx_frames;
  return result;
}

static int raspike_send_v2(RasPikePort port, int msg_id, const char *buf,
                           size_t size, uint32_t sequence,
                           unsigned char flags)
{
  if (!is_valid_protocol_port(port) || size > RP_V2_MAX_PAYLOAD
      || (size > 0 && !buf)) return E_PAR;
  unsigned char frame[RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD];
  frame[0] = RP_V2_START;
  frame[RP_V2_OFFSET_VERSION] = RP_V2_VERSION;
  frame[RP_V2_OFFSET_FLAGS] = flags;
  frame[RP_V2_OFFSET_CMD] = (unsigned char)msg_id;
  frame[RP_V2_OFFSET_PORT] = port;
  frame[RP_V2_OFFSET_SIZE] = (unsigned char)size;
  rp_v2_put_u32(frame + RP_V2_OFFSET_SEQUENCE, sequence);
  rp_v2_put_u32(frame + RP_V2_OFFSET_SESSION, fgSessionId);
  if (buf && size > 0) memcpy(frame + RP_V2_OFFSET_PAYLOAD, buf, size);
  size_t crc_offset = RP_V2_OFFSET_PAYLOAD + size;
  uint16_t crc = rp_v2_crc16(frame + 1, (RP_V2_HEADER_SIZE - 1u) + size);
  rp_v2_put_u16(frame + crc_offset, crc);

  const size_t frame_size = crc_offset + RP_V2_CRC_SIZE;
  const int result = (flags & RP_V2_FLAG_TELEMETRY)
      ? (int)serial_try_wri_dat(SIO_USB_PORTID,
                                (const char *)frame, frame_size)
      : enqueue_critical_tx(frame, frame_size);
  if (result >= 0) ++fgLinkStats.tx_frames;
  return result;
}

static int raspike_send_data_sequence(RasPikePort port, int msg_id,
                                      const char *buf, size_t size,
                                      uint32_t sequence,
                                      unsigned char flags)
{
  if (!fgProtocolV2Active) return raspike_send_legacy(port, msg_id, buf, size);
  return raspike_send_v2(port, msg_id, buf, size, sequence, flags);
}

static int raspike_send_data(RasPikePort port, int msg_id,
                             const char *buf, size_t size)
{
  unsigned char flags = 0;
  if (msg_id == RP_CMD_ID_ALL_STATUS
      || msg_id == RP_CMD_ID_DRIVE_TELEMETRY
      || msg_id == RP_CMD_ID_RUNTIME_TELEMETRY
      || msg_id == RP_CMD_ID_LINK_STATS) {
    flags |= RP_V2_FLAG_TELEMETRY;
  }
  return raspike_send_data_sequence(port, msg_id, buf, size,
                                    next_tx_sequence(), flags);
}

static void reset_link_session_preserve_devices(void)
{
  stp_cyc(APP_NOTIFY_CYC);
  stp_cyc(APP_DRIVE_TELEMETRY_CYC);
  stp_cyc(APP_SONER_CYC);
  stp_cyc(APP_DRIVE_CONTROL_4MS_CYC);
  stp_cyc(APP_DRIVE_CONTROL_10MS_CYC);
  stp_cyc(APP_RUNTIME_TELEMETRY_CYC);
  stop_all_motors_for_reset();
  lock_status();
  fgDriveConfigured = false;
  fgDriveRunning = false;
  fgDriveMotorsStopped = true;
  fgDriveTelemetrySequence = 0;
  fgDrivePreviousControlUs = 0;
  fgDriveIntegral = 0;
  fgDrivePreviousError = 0;
  fgDriveStatusFlags = 0;
  hub_runtime_reset(&fgRuntimeState);
  hub_fault_injection_clear(&fgRuntimeFault);
  fgRuntimeTelemetrySequence = 0;
  fgRuntimeSampleSequence = 0;
  fgRuntimeOverrunCount = 0;
  fgRuntimeRecoverRequested = false;
  fgDriveControlTaskPending = false;
  realtime_drive_reset_command_mailbox();
  realtime_drive_reset_controller_metrics();
  memset(&fgRuntimeTelemetrySnapshot, 0, sizeof(fgRuntimeTelemetrySnapshot));
  unlock_status();
}

static int begin_link_session(bool reset_runtime)
{
  static uint32_t session_counter = 0;
  if (reset_runtime) {
    reset_link_session_preserve_devices();
  } else {
    memset(fgDevices, 0, sizeof(fgDevices));
    memset(&fgCurrentStatus, 0, sizeof(fgCurrentStatus));
  }

  fgProtocolV2Active = false;
  fgSafetyInhibitReasons &= ~(RP_SAFETY_INHIBIT_USB | RP_SAFETY_INHIBIT_CONTROL);
  fgSafetyControlStaleCycles = 0u;
  fgSafetyLastControlUpdateCount = fgDriveControllerUpdateCount;
  fgSafetyUsbWasConnected = pbdrv_usb_is_connected();
  fgTxSequence = 1;
  fgActiveRequestSequence = 0;
  fgActiveRequestSession = 0;
  fgActiveRequestFlags = 0;
  fgActiveRequestCrc = 0;
  memset(&fgLinkStats, 0, sizeof(fgLinkStats));
  memset(fgAckCache, 0, sizeof(fgAckCache));

  SYSTIM session_time = 0;
  get_tim(&session_time);
  ++session_counter;
  fgSessionId = ((uint32_t)session_time ^ 0x52505632u
                 ^ (session_counter * 0x9e3779b9u)
                 ^ ((uint32_t)SPIKE_EXPECTED_VERSION_MINOR << 24));
  if (fgSessionId == 0) fgSessionId = session_counter ? session_counter : 1;

  for (int i = 0; i < RP_MAX_DEVICES; ++i) {
    fgCurrentStatus.ports[i].port = i;
  }

  unsigned char version[5] = {
    RP_CMD_INIT,
    RP_CMD_INIT_MAGIC,
    SPIKE_EXPECTED_VERSION_MAJOR,
    SPIKE_EXPECTED_VERSION_MINOR,
    SPIKE_EXPECTED_VERSION_PATCH
  };
  int result = enqueue_critical_tx(version, sizeof(version));
  if (result < 0) return result;

  unsigned char hello[RP_V2_HELLO_PAYLOAD_SIZE] = {0};
  hello[0] = RP_V2_VERSION;
  hello[1] = RP_V2_CAPABILITIES;
  rp_v2_put_u16(hello + 2, RP_V2_MAX_PAYLOAD);
  rp_v2_put_u32(hello + 4, fgSessionId);
  rp_v2_put_u32(hello + 8, 50u * 1000u);
  result = raspike_send_legacy(RP_PORT_NONE, RP_CMD_ID_LINK_HELLO,
                               (const char *)hello, sizeof(hello));
  if (result < 0) return result;

  fgProtocolV2Active = true;
  if (hub_imu_is_ready()) {
    fgRuntimeImuReadyLatched = true;
  }
  if (reset_runtime && fgRuntimeImuReadyLatched) {
    sta_cyc(APP_NOTIFY_CYC);
    sta_cyc(APP_DRIVE_TELEMETRY_CYC);
  }
  return 0;
}

static int raspike_receive_data(char *buf, size_t capacity,
                                RasPikePort *port, unsigned char *cmd,
                                unsigned char *data_size)
{
  unsigned char start = 0;
  for (;;) {
    int result = wait_read((char *)&start, 1);
    if (result < 0) return result;
    if (start == RP_CMD_INIT) {
      unsigned char magic = 0;
      result = wait_read((char *)&magic, 1);
      if (result < 0) return result;
      if (magic == RP_CMD_INIT_MAGIC) {
        result = begin_link_session(true);
        return result < 0 ? result : RP_LINK_RENEGOTIATED;
      }
      ++fgLinkStats.framing_errors;
      continue;
    }
    if (start == RP_CMD_START || start == RP_V2_START) break;
    ++fgLinkStats.framing_errors;
  }

  if (start == RP_CMD_START) {
    unsigned char header[3];
    int result = wait_read((char *)header, sizeof(header));
    if (result < 0) return result;
    *cmd = header[0];
    *data_size = header[1];
    *port = header[2];
    fgActiveRequestSequence = 0;
    fgActiveRequestSession = 0;
    fgActiveRequestFlags = 0;
    fgActiveRequestCrc = 0;

    if (*data_size > capacity || !is_valid_protocol_port(*port)) {
      discard_bytes(*data_size);
      ++fgLinkStats.framing_errors;
      return E_PAR;
    }
    if (*data_size > 0) {
      result = wait_read(buf, *data_size);
      if (result < 0) return result;
    }
    /* Compatibility fallback: legacy command frames remain accepted after
       v2 negotiation. Normal traffic still uses session/sequence/CRC. */
    ++fgLinkStats.rx_frames;
    return *data_size;
  }

  unsigned char frame[RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD];
  frame[0] = RP_V2_START;
  int result = wait_read((char *)frame + 1, RP_V2_HEADER_SIZE - 1u);
  if (result < 0) return result;

  unsigned char payload_size = frame[RP_V2_OFFSET_SIZE];
  if (payload_size > RP_V2_MAX_PAYLOAD || payload_size > capacity) {
    discard_bytes((size_t)payload_size + RP_V2_CRC_SIZE);
    ++fgLinkStats.framing_errors;
    return E_PAR;
  }
  result = wait_read((char *)frame + RP_V2_OFFSET_PAYLOAD,
                     (size_t)payload_size + RP_V2_CRC_SIZE);
  if (result < 0) return result;

  size_t crc_offset = RP_V2_OFFSET_PAYLOAD + payload_size;
  uint16_t received_crc = rp_v2_get_u16(frame + crc_offset);
  uint16_t calculated_crc = rp_v2_crc16(
      frame + 1, (RP_V2_HEADER_SIZE - 1u) + payload_size);
  if (received_crc != calculated_crc) {
    ++fgLinkStats.crc_errors;
    return E_SYS;
  }
  if (frame[RP_V2_OFFSET_VERSION] != RP_V2_VERSION) {
    ++fgLinkStats.framing_errors;
    return E_NOSPT;
  }

  *cmd = frame[RP_V2_OFFSET_CMD];
  *port = frame[RP_V2_OFFSET_PORT];
  *data_size = payload_size;
  fgActiveRequestFlags = frame[RP_V2_OFFSET_FLAGS];
  fgActiveRequestSequence = rp_v2_get_u32(frame + RP_V2_OFFSET_SEQUENCE);
  fgActiveRequestSession = rp_v2_get_u32(frame + RP_V2_OFFSET_SESSION);
  fgActiveRequestCrc = received_crc;

  if (!is_valid_protocol_port(*port)) {
    ++fgLinkStats.framing_errors;
    return E_PAR;
  }
  if (!fgProtocolV2Active || fgActiveRequestSession != fgSessionId) {
    ++fgLinkStats.session_errors;
    return E_OBJ;
  }
  if (replay_cached_ack(*port, *cmd, fgActiveRequestSequence,
                        fgActiveRequestSession, fgActiveRequestCrc)) {
    return RP_REQUEST_REPLAYED;
  }
  if (payload_size > 0) memcpy(buf, frame + RP_V2_OFFSET_PAYLOAD, payload_size);
  ++fgLinkStats.rx_frames;
  return payload_size;
}


#if 0
static void update_port(RasPikePort port, const int cmd_id,char *data, size_t data_size)
{
  lock_status();
  fgCurrentStatus.ports[port].cmd = (char)cmd_id;
  fgCurrentStatus.ports[port].port = port;
  memset(fgCurrentStatus.ports[port].data,0,sizeof(fgCurrentStatus.ports[port].data));
  if ( data ) {
    memcpy(fgCurrentStatus.ports[port].data,data,data_size);
  }
  unlock_status();
}
#endif

static void update_port_config(RasPikePort port, const int type, void *device)
{
  if (port >= RP_MAX_DEVICES || !device) {
    ++fgLinkStats.invalid_commands;
    return;
  }
  lock_status();
  fgDevices[port].config = type;
  fgDevices[port].device = device;
  unlock_status();
}

static void update_device_cmd(RasPikePort port, const int cmd_id, char *data, size_t data_size)
{
  (void)data;
  (void)data_size;
  /* Publish the delayed command under the short status lock. Sensor I/O is
     deliberately kept outside this critical section. */
  lock_status();
  fgDevices[port].next_cmd = cmd_id;
  unlock_status();
  /*
  RPProtocolPortStatus *p = &fgCurrentStatus.ports[port];
  memset(p,0,sizeof(*p));
  p->port = (char)port;
  p->cmd = (char)cmd_id;
  memset(p->data,0,sizeof(p->data));
  if ( p->data ) {
    memcpy(p->data,data,data_size);
  }
  */
}

static int ack_cache_index(RasPikePort port)
{
  if (port == RP_PORT_NONE) return RP_MAX_DEVICES;
  return port < RP_MAX_DEVICES ? (int)port : -1;
}

static int replay_cached_ack(RasPikePort port, unsigned char command,
                             uint32_t sequence, uint32_t session,
                             uint16_t request_crc)
{
  int index = ack_cache_index(port);
  if (index < 0 || sequence == 0) return 0;
  RPCachedAck *cached = &fgAckCache[index];
  if (!cached->valid || cached->session != session
      || cached->sequence != sequence || cached->command != command
      || cached->request_crc != request_crc) {
    return 0;
  }
  int32_t payload[2] = {(int32_t)command, cached->result};
  unsigned char flags = RP_V2_FLAG_ACK;
  if (cached->result < 0) flags |= RP_V2_FLAG_ERROR;
  (void)raspike_send_data_sequence(port, RP_CMD_ID_ACK,
                                    (const char *)payload, sizeof(payload),
                                    sequence, flags);
  ++fgLinkStats.duplicate_requests;
  return 1;
}

static int send_ack_sequence_with_ticket(RasPikePort port, const char cmd_id,
                                         int32_t data, uint32_t sequence,
                                         uint32_t *ticket_out)
{
  int index = ack_cache_index(port);
  if (fgProtocolV2Active && index >= 0 && sequence != 0) {
    fgAckCache[index].session = fgSessionId;
    fgAckCache[index].sequence = sequence;
    fgAckCache[index].command = (unsigned char)cmd_id;
    fgAckCache[index].request_crc = fgActiveRequestCrc;
    fgAckCache[index].result = data;
    fgAckCache[index].valid = true;
  }
  int32_t payload[2] = {(int32_t)cmd_id, data};
  unsigned char flags = RP_V2_FLAG_ACK;
  if (data < 0) flags |= RP_V2_FLAG_ERROR;
  if (!fgProtocolV2Active) {
    return raspike_send_legacy(port, RP_CMD_ID_ACK,
                               (const char *)payload, sizeof(payload));
  }

  unsigned char frame[RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD];
  frame[0] = RP_V2_START;
  frame[RP_V2_OFFSET_VERSION] = RP_V2_VERSION;
  frame[RP_V2_OFFSET_FLAGS] = flags;
  frame[RP_V2_OFFSET_CMD] = RP_CMD_ID_ACK;
  frame[RP_V2_OFFSET_PORT] = port;
  frame[RP_V2_OFFSET_SIZE] = sizeof(payload);
  rp_v2_put_u32(frame + RP_V2_OFFSET_SEQUENCE, sequence);
  rp_v2_put_u32(frame + RP_V2_OFFSET_SESSION, fgSessionId);
  memcpy(frame + RP_V2_OFFSET_PAYLOAD, payload, sizeof(payload));
  const size_t crc_offset = RP_V2_OFFSET_PAYLOAD + sizeof(payload);
  const uint16_t crc = rp_v2_crc16(
      frame + 1, (RP_V2_HEADER_SIZE - 1u) + sizeof(payload));
  rp_v2_put_u16(frame + crc_offset, crc);
  const int result = enqueue_critical_tx_with_ticket(
      frame, crc_offset + RP_V2_CRC_SIZE, ticket_out);
  if (result >= 0) ++fgLinkStats.tx_frames;
  return result;
}

static void send_ack_sequence(RasPikePort port, const char cmd_id,
                              int32_t data, uint32_t sequence)
{
  (void)send_ack_sequence_with_ticket(port, cmd_id, data, sequence, NULL);
}

static void send_ack(RasPikePort port, const char cmd_id, int32_t data)
{
  send_ack_sequence(port, cmd_id, data, fgActiveRequestSequence);
}

static bool send_maintenance_ack_and_wait(RasPikePort port,
                                          const char cmd_id,
                                          int32_t data)
{
  uint32_t ticket = 0u;
  if (send_ack_sequence_with_ticket(port, cmd_id, data,
                                    fgActiveRequestSequence, &ticket) < 0) {
    return false;
  }
  return wait_for_critical_tx(ticket, RP_MAINTENANCE_TX_TIMEOUT_US);
}


void update_hub_status(RPProtocolSpikeStatus *status)
{
  hub_button_t button;
  status->voltage = hub_battery_get_voltage();
  status->current = hub_battery_get_current();
  hub_button_is_pressed(&button);
  status->button = button;
  hub_imu_get_angular_velocity(status->angular_velocity);
  hub_imu_get_acceleration(status->acceleration);
  status->is_ready = hub_imu_is_ready();
  status->is_statinary = hub_imu_is_stationary();
  if ( status->is_ready ) {
    status->heading = hub_imu_get_heading();
  } else {
    status->heading = 0.0f;
  }

}

void update_port_device_colorsensor(unsigned char cmd_id,pup_device_t *dev,RPProtocolPortStatus *status)
{
 switch (cmd_id) {  
    case RP_CMD_ID_COL_RGB:
    {
      pup_color_rgb_t rgb = pup_color_sensor_rgb(dev);
      memcpy(status->data,&rgb,sizeof(rgb));
    }
    break;
    case RP_CMD_ID_COL_COL:
    case RP_CMD_ID_COL_COL_SUR_OFF:
    {
      bool surface = (cmd_id == RP_CMD_ID_COL_COL);  
      pup_color_hsv_t col = pup_color_sensor_color(dev,surface);
      memcpy(status->data,&col,sizeof(col));
    }
    break;
    case RP_CMD_ID_COL_HSV:
    case RP_CMD_ID_COL_HSV_SUR_OFF:
    {
      bool surface = (cmd_id == RP_CMD_ID_COL_HSV);
      pup_color_hsv_t hsv;
      hsv = pup_color_sensor_hsv(dev,surface);
      memcpy(status->data,&hsv,sizeof(hsv));
    }
    break;
    case RP_CMD_ID_COL_REF:
    {
      int32_t ref = pup_color_sensor_reflection(dev);
      memcpy(status->data,&ref,sizeof(ref));
    }
    break;
    case RP_CMD_ID_COL_AMB:
    {
      int32_t amb = pup_color_sensor_ambient(dev);
      memcpy(status->data,&amb,sizeof(amb));
    }
    break;
    default:
    break;
  
  }



}

void update_port_device_forcesensor(unsigned char cmd_id,pup_device_t *dev,RPProtocolPortStatus *status)
{
  // TODO: maybe need update frequency
  float force = pup_force_sensor_force(dev);
  float distance = pup_force_sensor_distance(dev);
  bool touched = pup_force_sensor_touched(dev);

  memcpy(status->data+RP_FORCESENSOR_INDEX_FRC,&force,sizeof(force));
  memcpy(status->data+RP_FORCESENSOR_INDEX_DST,&distance,sizeof(distance));
  memcpy(status->data+RP_FORCESENSOR_INDEX_TCH,&touched,sizeof(touched));

}

void update_port_device_motor(unsigned char cmd_id, pup_motor_t *dev, RPProtocolPortStatus *status)
{
  // setupする前にgetするとSPIKEが死んでしまう
  if ( cmd_id != RP_CMD_ID_MOT_STU ) 
    return;

  int32_t count = pup_motor_get_count(dev);
  int32_t speed = pup_motor_get_speed(dev);
  int16_t pow   = (int16_t)pup_motor_get_power(dev);
  bool is_stalled = pup_motor_is_stalled(dev);
  memcpy(status->data+RP_MOTOR_INDEX_COUNT,&count,sizeof(count));
  memcpy(status->data+RP_MOTOR_INDEX_SPEED,&speed,sizeof(speed));
  memcpy(status->data+RP_MOTOR_INDEX_POWER,&pow,sizeof(pow));
  memcpy(status->data+RP_MOTOR_INDEX_ISSTALLED,&is_stalled,sizeof(is_stalled));
  #if 0
  *(int32_t*)&status->data[RP_MOTOR_INDEX_COUNT] = pup_motor_get_count(dev);
  *(int32_t*)&status->data[RP_MOTOR_INDEX_SPEED] = pup_motor_get_speed(dev);
  *(int16_t*)&status->data[RP_MOTOR_INDEX_POWER] = (int16_t)pup_motor_get_power(dev);
  *(bool *)&status->data[RP_MOTOR_INDEX_ISSTALLED] = pup_motor_is_stalled(dev);
#endif
}

void update_port_device_ultrasonicsensor(unsigned char cmd_id,pup_device_t *dev,RPProtocolPortStatus *status)
{
//  static int count = 0;
//  if ( (count++ % 10) != 0 ) return;
  int32_t distance = pup_ultrasonic_sensor_distance(dev);
  // calling presence break distance to 0. so do not support presence 
  //bool presence = pup_ultrasonic_sensor_presence(dev);
//  hub_display_number(distance);
  memcpy(status->data + RP_US_INDEX_DISTANCE, &distance, sizeof(distance));
}


void update_port_device(RPDevice *device,RPProtocolPortStatus *status)
{
  // デバイスが設定されていない時は何もしない
  void *dev = device->device;
  if ( !dev ) {
    return;
  }
  unsigned char cmd_id = status->cmd;
  unsigned char next_cmd = device->next_cmd;

  /* 遅延コマンドの処理。遅延コマンドが指定されていた場合は、それに上書きする*/
  if (next_cmd && next_cmd != cmd_id) {
    /* update command*/
    cmd_id = status->cmd = next_cmd;
    device->next_cmd = 0;
  }
 
  switch(device->config) {
    case RP_CMD_TYPE_COLOR:
      update_port_device_colorsensor(cmd_id,(pup_device_t*)dev,status);
      break;
    case RP_CMD_TYPE_FORCE:
      update_port_device_forcesensor(cmd_id,(pup_device_t*)dev,status);
      break;
    case RP_CMD_TYPE_MOTOR:
      update_port_device_motor(cmd_id, (pup_motor_t *)dev, status);
      break;
    case RP_CMD_TYPE_US:
//      updading ultrasonic is executed by another task
//      update_port_device_ultrasonicsensor(cmd_id,(pup_device_t*)dev,status);
      break;    
    default:
      break;
  }
  

}

void update_port_devices(RPDevice *devices, int num, RPProtocolSpikeStatus *status)
{
  int i;
  for ( i = 0 ; i < num; i++ ) {
    void *dev = devices[i].device;
    if ( dev ) {
//      hub_display_number(i);
      update_port_device(devices+i,status->ports+i);
    }
  }
}

void update_ultrasonicsensor_port_devices(RPDevice *devices, int num, RPProtocolSpikeStatus *status)
{
  int i;
  lock_status();
  for ( i = 0 ; i < num; i++ ) {
    void *dev = devices[i].device;
    if ( dev && devices[i].config == RP_CMD_TYPE_US ) {
      update_port_device_ultrasonicsensor((status->ports+i)->cmd, dev, status->ports+i);
    }
  }
  unlock_status();
}





static void stop_all_motors_for_reset(void)
{
  hub_emergency_stop_all(true);
}

static bool hub_maintenance_transition(RasPikePort port, int command_id,
                                       pbdrv_reset_action_t action)
{
  if (!pbdrv_reset_action_is_supported(action)) {
    send_ack(port, command_id, 0);
    return false;
  }

  /* Reject in-flight controller output before stopping the hardware. The main
     command task is occupied until reset, so no later drive command can pass. */
  ++fgSafetyGeneration;
  fgSafetyInhibitReasons |= RP_SAFETY_INHIBIT_CONTROL;
  stop_all_motors_for_reset();
  stp_cyc(APP_DRIVE_CONTROL_4MS_CYC);
  stp_cyc(APP_DRIVE_CONTROL_10MS_CYC);

  if (!send_maintenance_ack_and_wait(port, command_id, 1)) {
    pbdrv_reset(PBDRV_RESET_ACTION_RESET);
  }

  reset_runtime_state_for_restart();
  pbdrv_reset(action);
  return true;
}

static void reset_runtime_state_for_restart(void)
{
  stp_cyc(APP_NOTIFY_CYC);
  stp_cyc(APP_DRIVE_TELEMETRY_CYC);
  stp_cyc(APP_SONER_CYC);
  stp_cyc(APP_DRIVE_CONTROL_4MS_CYC);
  stp_cyc(APP_DRIVE_CONTROL_10MS_CYC);
  stp_cyc(APP_RUNTIME_TELEMETRY_CYC);
  stop_all_motors_for_reset();
  lock_status();
  fgDriveConfigured = false;
  fgDriveRunning = false;
  fgDriveMotorsStopped = true;
  fgDriveLeftMotor = NULL;
  fgDriveRightMotor = NULL;
  fgDriveColorSensor = NULL;
  fgDriveTelemetrySequence = 0;
  fgDrivePreviousControlUs = 0;
  fgDriveIntegral = 0;
  fgDrivePreviousError = 0;
  fgDriveStatusFlags = 0;
  hub_runtime_reset(&fgRuntimeState);
  hub_fault_injection_clear(&fgRuntimeFault);
  fgRuntimeTelemetrySequence = 0;
  fgRuntimeSampleSequence = 0;
  fgRuntimeOverrunCount = 0;
  fgRuntimeRecoverRequested = false;
  fgDriveControlTaskPending = false;
  realtime_drive_reset_command_mailbox();
  realtime_drive_reset_controller_metrics();
  memset(&fgRuntimeTelemetrySnapshot, 0, sizeof(fgRuntimeTelemetrySnapshot));
  memset(&fgCurrentStatus, 0, sizeof(fgCurrentStatus));
  memset(fgDevices, 0, sizeof(fgDevices));
  unlock_status();
}

static void process_sys_cmd(RasPikePort port, const int cmd_id, char *param)
{
  switch (cmd_id ) {
    case RP_CMD_ID_SHT_DWN:
      hub_system_shutdown();
      // Not reached
      break;
    case RP_CMD_ID_DRIVE_CONFIG:
    {
      loc_mtx(APP_DRIVE_STATE_MUTEX);
      int32_t success = realtime_drive_configure_devices();
      unl_mtx(APP_DRIVE_STATE_MUTEX);
      send_ack(port, cmd_id, success);
      break;
    }
    case RP_CMD_ID_DRIVE_COMMAND:
    {
      RPRealtimeDriveCommand command;
      memcpy(&command, param, sizeof(command));
      loc_mtx(APP_DRIVE_STATE_MUTEX);
      realtime_drive_select_control_period(command.control_period_us);
      realtime_drive_publish_command(&command, realtime_drive_now_ms());
      lock_status();
      fgDriveStatusFlags &= ~RP_DRIVE_STATUS_WATCHDOG_STOP;
      const bool safety_inhibited = fgSafetyInhibitReasons != 0u;
      if (command.flags & RP_DRIVE_FLAG_STOP || command.mode == RP_DRIVE_MODE_SAFE_STOP) {
        realtime_drive_stop(false);
      } else if (safety_inhibited) {
        realtime_drive_stop(false);
        fgDriveStatusFlags |= RP_DRIVE_STATUS_WATCHDOG_STOP;
      } else if ((command.flags & RP_DRIVE_FLAG_ARM) && fgDriveConfigured) {
        fgDriveStatusFlags |= RP_DRIVE_STATUS_ARMED;
        fgRuntimeRecoverRequested = true;
      }
      if ((command.flags & RP_DRIVE_FLAG_START) && fgDriveConfigured
          && !safety_inhibited) {
        fgDriveRunning = true;
        fgDriveMotorsStopped = false;
        fgDriveIntegral = 0;
        fgDrivePreviousError = 0;
        realtime_drive_reset_controller_metrics();
        fgDriveStatusFlags |= RP_DRIVE_STATUS_RUNNING;
      }
      unlock_status();
      unl_mtx(APP_DRIVE_STATE_MUTEX);
      break;
    }
    case RP_CMD_ID_RUNTIME_FAULT:
    {
      RPRuntimeFaultCommand command;
      memcpy(&command, param, sizeof(command));
      loc_mtx(APP_DRIVE_STATE_MUTEX);
      lock_status();
      if (command.fault_mask == 0u || command.duration_cycles == 0u) {
        hub_fault_injection_clear(&fgRuntimeFault);
      } else {
        hub_fault_injection_start(&fgRuntimeFault,
                                  command.fault_mask,
                                  command.duration_cycles);
      }
      unlock_status();
      unl_mtx(APP_DRIVE_STATE_MUTEX);
      send_ack(port, cmd_id, 1);
      break;
    }
    case RP_CMD_ID_LINK_PING:
      send_ack(port, cmd_id, 1);
      break;
    case RP_CMD_ID_LINK_STATS:
      send_ack(port, cmd_id, 1);
      raspike_send_data(RP_PORT_NONE, RP_CMD_ID_LINK_STATS,
                        (const char *)&fgLinkStats, sizeof(fgLinkStats));
      break;
    case RP_CMD_ID_UPDATE_MODE:
    {
      uint32_t magic = 0;
      memcpy(&magic, param, sizeof(magic));
      if (magic != RP_UPDATE_MODE_MAGIC) {
        send_ack(port, cmd_id, 0);
        break;
      }
      (void)hub_maintenance_transition(
          port, cmd_id, PBDRV_RESET_ACTION_RESET_IN_UPDATE_MODE);
      break;
    }
    case RP_CMD_ID_SOFT_RST:
    {
      uint32_t magic = 0;
      memcpy(&magic, param, sizeof(magic));
      if (magic != RP_SOFT_RESET_MAGIC) {
        send_ack(port, cmd_id, 0);
        break;
      }

      (void)hub_maintenance_transition(
          port, cmd_id, PBDRV_RESET_ACTION_RESET);
      // Not reached when the action is supported.
      break;
    }
    default:
      break;
  }


}

static void process_color_sensor_cmd(RasPikePort port, int cmd_id,
                                     const char *param)
{
  switch (cmd_id) {
    case RP_CMD_ID_COL_CFG:
      if (fgDevices[port].device) {
        if (fgDevices[port].config == RP_CMD_TYPE_COLOR) {
          ++fgLinkStats.duplicate_configs;
          send_ack(port, cmd_id, 1);
        } else {
          send_ack(port, cmd_id, RP_LINK_ERR_DEVICE_CONFLICT);
        }
        return;
      }
      {
        pup_device_t *device = pup_color_sensor_get_device(PORT_FROM_RASPIKE(port));
        if (!device) {
          send_ack(port, cmd_id, 0);
          return;
        }
        lock_status();
        update_port_config(port, RP_CMD_TYPE_COLOR, device);
        unlock_status();
        send_ack(port, cmd_id, 1);
      }
      return;

    case RP_CMD_ID_COL_RGB:
    case RP_CMD_ID_COL_COL:
    case RP_CMD_ID_COL_COL_SUR_OFF:
    case RP_CMD_ID_COL_HSV:
    case RP_CMD_ID_COL_HSV_SUR_OFF:
    case RP_CMD_ID_COL_REF:
    case RP_CMD_ID_COL_AMB:
      if (fgCurrentStatus.ports[port].cmd != (char)cmd_id) {
        update_device_cmd(port, cmd_id, NULL, 0);
      }
      return;

    case RP_CMD_ID_COL_LIGHT_SET: {
      int32_t brightness[3] = {0};
      memcpy(brightness, param, sizeof(brightness));
      pbio_error_t error = pup_color_sensor_light_set(
          (pup_device_t *)fgDevices[port].device,
          brightness[0], brightness[1], brightness[2]);
      send_ack(port, cmd_id, error == PBIO_SUCCESS ? 1 : 0);
      return;
    }

    case RP_CMD_ID_COL_LIGHT_ON:
      (void)pup_color_sensor_light_on((pup_device_t *)fgDevices[port].device);
      return;

    case RP_CMD_ID_COL_LIGHT_OFF:
      (void)pup_color_sensor_light_off((pup_device_t *)fgDevices[port].device);
      return;

    default:
      ++fgLinkStats.invalid_commands;
      return;
  }
}

static void process_force_sensor_cmd(RasPikePort port, int cmd_id,
                                     const char *param)
{
  (void)param;
  if (cmd_id != RP_CMD_ID_FRC_CFG) {
    ++fgLinkStats.invalid_commands;
    return;
  }
  if (fgDevices[port].device) {
    if (fgDevices[port].config == RP_CMD_TYPE_FORCE) {
      ++fgLinkStats.duplicate_configs;
      send_ack(port, cmd_id, 1);
    } else {
      send_ack(port, cmd_id, RP_LINK_ERR_DEVICE_CONFLICT);
    }
    return;
  }

  pup_device_t *device = pup_force_sensor_get_device(PORT_FROM_RASPIKE(port));
  if (!device) {
    send_ack(port, cmd_id, 0);
    return;
  }
  lock_status();
  update_port_config(port, RP_CMD_TYPE_FORCE, device);
  unlock_status();
  send_ack(port, cmd_id, 1);
}

static void process_motor_cmd(RasPikePort port, int cmd_id, const char *param)
{
  pup_motor_t *motor = (pup_motor_t *)fgDevices[port].device;
  switch (cmd_id) {
    case RP_CMD_ID_MOT_CFG:
      if (motor) {
        if (fgDevices[port].config == RP_CMD_TYPE_MOTOR) {
          ++fgLinkStats.duplicate_configs;
          send_ack(port, cmd_id, 1);
        } else {
          send_ack(port, cmd_id, RP_LINK_ERR_DEVICE_CONFLICT);
        }
        return;
      }
      /* Configuration is one-shot and bounded to 200 ms. Performing it in
         the command task eliminates the pending-mask lost-wakeup race and
         makes the ACK a strict physical-device-ready guarantee. */
      motor = pup_motor_get_device(PORT_FROM_RASPIKE(port));
      if (!motor) {
        ++fgLinkStats.invalid_commands;
        send_ack(port, cmd_id, 0);
        return;
      }
      lock_status();
      update_port_config(port, RP_CMD_TYPE_MOTOR, motor);
      unlock_status();
      send_ack(port, cmd_id, 1);
      return;

    case RP_CMD_ID_MOT_STU: {
      pup_direction_t direction;
      memcpy(&direction, param + RP_MOTOR_STU_INDEX_DIRECTION,
             sizeof(direction));
      bool reset = read_bool_value(param + RP_MOTOR_STU_INDEX_RESETCOUNT);
      pbio_error_t error = pup_motor_setup(motor, direction, reset);
      if (error != PBIO_SUCCESS) {
        send_ack(port, cmd_id, 0);
        return;
      }
      update_device_cmd(port, cmd_id, NULL, 0);
      if (reset) {
        const int32_t zero = 0;
        lock_status();
        memcpy(fgCurrentStatus.ports[port].data + RP_MOTOR_INDEX_COUNT,
               &zero, sizeof(zero));
        unlock_status();
      }
      send_ack(port, cmd_id, 1);
      return;
    }

    case RP_CMD_ID_MOT_RST: {
      pbio_error_t error = pup_motor_reset_count(motor);
      if (error == PBIO_SUCCESS) {
        const int32_t zero = 0;
        lock_status();
        memcpy(fgCurrentStatus.ports[port].data + RP_MOTOR_INDEX_COUNT,
               &zero, sizeof(zero));
        unlock_status();
      }
      send_ack(port, cmd_id, error == PBIO_SUCCESS ? 1 : 0);
      return;
    }

    case RP_CMD_ID_MOT_SPD:
      if (fgSafetyInhibitReasons != 0u) {
        hub_emergency_stop_all(true);
        return;
      }
      (void)pup_motor_set_speed(motor, read_i32(param));
      return;

    case RP_CMD_ID_MOT_POW:
      if (fgSafetyInhibitReasons != 0u) {
        hub_emergency_stop_all(true);
        return;
      }
      (void)pup_motor_set_power(motor, read_i32(param));
      return;

    case RP_CMD_ID_MOT_STP: {
      pbio_error_t error = pup_motor_stop(motor);
      send_ack(port, cmd_id, error == PBIO_SUCCESS ? 1 : 0);
      return;
    }

    case RP_CMD_ID_MOT_STP_BRK: {
      pbio_error_t error = pup_motor_brake(motor);
      send_ack(port, cmd_id, error == PBIO_SUCCESS ? 1 : 0);
      return;
    }

    case RP_CMD_ID_MOT_STP_HLD: {
      if (fgSafetyInhibitReasons != 0u) {
        hub_emergency_stop_all(
            (fgSafetyInhibitReasons & RP_SAFETY_INHIBIT_BATTERY) == 0u);
        send_ack(port, cmd_id, 0);
        return;
      }
      pbio_error_t error = pup_motor_hold(motor);
      send_ack(port, cmd_id, error == PBIO_SUCCESS ? 1 : 0);
      return;
    }

    case RP_CMD_ID_MOT_SET_DTY: {
      int32_t old_value = pup_motor_set_duty_limit(motor, read_i32(param));
      send_ack(port, cmd_id, old_value);
      return;
    }

    case RP_CMD_ID_MOT_RST_DTY:
      pup_motor_restore_duty_limit(motor, read_i32(param));
      send_ack(port, cmd_id, 1);
      return;

    default:
      ++fgLinkStats.invalid_commands;
      return;
  }
}

static void process_ultrasonic_sensor_cmd(RasPikePort port, int cmd_id,
                                          const char *param)
{
  pup_device_t *device = (pup_device_t *)fgDevices[port].device;
  switch (cmd_id) {
    case RP_CMD_ID_US_CFG:
      if (device) {
        if (fgDevices[port].config == RP_CMD_TYPE_US) {
          ++fgLinkStats.duplicate_configs;
          send_ack(port, cmd_id, 1);
        } else {
          send_ack(port, cmd_id, RP_LINK_ERR_DEVICE_CONFLICT);
        }
        return;
      }
      device = pup_ultrasonic_sensor_get_device(PORT_FROM_RASPIKE(port));
      if (!device) {
        send_ack(port, cmd_id, 0);
        return;
      }
      lock_status();
      update_port_config(port, RP_CMD_TYPE_US, device);
      unlock_status();
      sta_cyc(APP_SONER_CYC);
      send_ack(port, cmd_id, 1);
      return;

    case RP_CMD_ID_US_LGT_SET: {
      int32_t brightness[4] = {0};
      memcpy(brightness, param, sizeof(brightness));
      (void)pup_ultrasonic_sensor_light_set(
          device, brightness[0], brightness[1], brightness[2], brightness[3]);
      return;
    }

    case RP_CMD_ID_US_LGT_ON:
      (void)pup_ultrasonic_sensor_light_on(device);
      return;

    case RP_CMD_ID_US_LGT_OFF:
      (void)pup_ultrasonic_sensor_light_off(device);
      return;

    default:
      ++fgLinkStats.invalid_commands;
      return;
  }
}

static void process_hub_cmd(RasPikePort port, int cmd_id, const char *param)
{
  (void)port;
  switch (cmd_id) {
    case RP_CMD_ID_HUB_IMU_RST_HDG:
      hub_imu_reset_heading();
      return;

    case RP_CMD_ID_HUB_IMU_SET_TLT:
      hub_imu_set_tilt(read_float32(param));
      return;

    case RP_CMD_ID_HUB_DISP_ORI:
      hub_display_orientation((uint8_t)param[0]);
      return;

    case RP_CMD_ID_HUB_DISP_OFF:
      hub_display_off();
      return;

    case RP_CMD_ID_HUB_DISP_PIX:
      hub_display_pixel((uint8_t)param[RP_HUB_DISP_PIX_INDEX_ROW],
                        (uint8_t)param[RP_HUB_DISP_PIX_INDEX_COL],
                        (uint8_t)param[RP_HUB_DISP_PIX_INDEX_BRT]);
      return;

    case RP_CMD_ID_HUB_DISP_IMG:
      hub_display_image((uint8_t *)param);
      return;

    case RP_CMD_ID_HUB_DISP_NUM:
      hub_display_number((int8_t)param[0]);
      return;

    case RP_CMD_ID_HUB_DISP_CHR:
      hub_display_char(param[0]);
      return;

    case RP_CMD_ID_HUB_DISP_TXT:
      hub_display_text(param + RP_HUB_DISP_TXT_INDEX_TXT,
                       read_u32(param + RP_HUB_DISP_TXT_INDEX_ON),
                       read_u32(param + RP_HUB_DISP_TXT_INDEX_OFF));
      return;

    case RP_CMD_ID_HUB_DISP_TXT_SCR:
      hub_display_text_scroll(param + RP_HUB_DISP_TXT_SCR_INDEX_TXT,
                              read_u32(param + RP_HUB_DISP_TXT_SCR_INDEX_DLY));
      return;

    case RP_CMD_ID_HUB_LGT_ON_HSV: {
      pbio_color_hsv_t hsv;
      memcpy(&hsv, param, sizeof(hsv));
      hub_light_on_hsv(&hsv);
      return;
    }

    case RP_CMD_ID_HUB_LGT_ON_COL: {
      pbio_color_t color;
      memcpy(&color, param, sizeof(color));
      hub_light_on_color(color);
      return;
    }

    case RP_CMD_ID_HUB_LGT_OFF:
      hub_light_off();
      return;

    case RP_CMD_ID_HUB_SPK_SET_VOL:
      hub_speaker_set_volume((uint8_t)param[0]);
      return;

    case RP_CMD_ID_HUB_SPK_PLY_TON:
      hub_speaker_play_tone(
          read_u16(param + RP_HUB_SPK_PLY_TON_INDEX_FRQ),
          read_i32(param + RP_HUB_SPK_PLY_TON_INDEX_DUR));
      return;

    case RP_CMD_ID_HUB_SPK_STP:
      hub_speaker_stop();
      return;

    default:
      ++fgLinkStats.invalid_commands;
      return;
  }
}

static int command_expects_ack(int cmd_id)
{
  switch (cmd_id) {
    case RP_CMD_ID_DRIVE_CONFIG:
    case RP_CMD_ID_SOFT_RST:
    case RP_CMD_ID_UPDATE_MODE:
    case RP_CMD_ID_LINK_PING:
    case RP_CMD_ID_LINK_STATS:
    case RP_CMD_ID_COL_CFG:
    case RP_CMD_ID_COL_LIGHT_SET:
    case RP_CMD_ID_FRC_CFG:
    case RP_CMD_ID_MOT_CFG:
    case RP_CMD_ID_MOT_STU:
    case RP_CMD_ID_MOT_RST:
    case RP_CMD_ID_MOT_STP:
    case RP_CMD_ID_MOT_STP_BRK:
    case RP_CMD_ID_MOT_STP_HLD:
    case RP_CMD_ID_MOT_SET_DTY:
    case RP_CMD_ID_MOT_RST_DTY:
    case RP_CMD_ID_US_CFG:
      return 1;
    default:
      return 0;
  }
}

static int validate_command(RasPikePort port, int cmd_id,
                            const char *param, size_t size)
{
  int type = GET_CMD_TYPE(cmd_id);
  if ((type == RP_CMD_TYPE_SYS || type == RP_CMD_TYPE_HUB)) {
    if (port != RP_PORT_NONE) return RP_LINK_ERR_INVALID_PORT;
  } else if (port >= RP_MAX_DEVICES) {
    return RP_LINK_ERR_INVALID_PORT;
  }
  if (size > 0 && !param) return RP_LINK_ERR_INVALID_SIZE;

  int expected = -1;
  switch (cmd_id) {
    case RP_CMD_ID_SHT_DWN:
    case RP_CMD_ID_DRIVE_CONFIG:
    case RP_CMD_ID_LINK_PING:
    case RP_CMD_ID_LINK_STATS:
    case RP_CMD_ID_COL_CFG:
    case RP_CMD_ID_COL_RGB:
    case RP_CMD_ID_COL_COL:
    case RP_CMD_ID_COL_COL_SUR_OFF:
    case RP_CMD_ID_COL_HSV:
    case RP_CMD_ID_COL_HSV_SUR_OFF:
    case RP_CMD_ID_COL_REF:
    case RP_CMD_ID_COL_AMB:
    case RP_CMD_ID_COL_LIGHT_ON:
    case RP_CMD_ID_COL_LIGHT_OFF:
    case RP_CMD_ID_FRC_CFG:
    case RP_CMD_ID_MOT_CFG:
    case RP_CMD_ID_MOT_RST:
    case RP_CMD_ID_MOT_STP:
    case RP_CMD_ID_MOT_STP_BRK:
    case RP_CMD_ID_MOT_STP_HLD:
    case RP_CMD_ID_US_CFG:
    case RP_CMD_ID_US_LGT_ON:
    case RP_CMD_ID_US_LGT_OFF:
    case RP_CMD_ID_HUB_DISP_OFF:
    case RP_CMD_ID_HUB_LGT_OFF:
    case RP_CMD_ID_HUB_SPK_STP:
    case RP_CMD_ID_HUB_IMU_RST_HDG:
      expected = 0;
      break;
    case RP_CMD_ID_SOFT_RST:
    case RP_CMD_ID_UPDATE_MODE:
      expected = 4;
      break;
    case RP_CMD_ID_DRIVE_COMMAND: expected = sizeof(RPRealtimeDriveCommand); break;
    case RP_CMD_ID_RUNTIME_FAULT: expected = sizeof(RPRuntimeFaultCommand); break;
    case RP_CMD_ID_COL_LIGHT_SET: expected = 12; break;
    case RP_CMD_ID_MOT_STU: expected = RP_MOTOR_STU_INDEX_RESETCOUNT + sizeof(bool); break;
    case RP_CMD_ID_MOT_SPD:
    case RP_CMD_ID_MOT_POW:
    case RP_CMD_ID_MOT_SET_DTY:
    case RP_CMD_ID_MOT_RST_DTY:
    case RP_CMD_ID_HUB_IMU_SET_TLT:
      expected = 4;
      break;
    case RP_CMD_ID_US_LGT_SET: expected = 16; break;
    case RP_CMD_ID_HUB_DISP_ORI:
    case RP_CMD_ID_HUB_DISP_NUM:
    case RP_CMD_ID_HUB_DISP_CHR:
    case RP_CMD_ID_HUB_SPK_SET_VOL:
      expected = 1;
      break;
    case RP_CMD_ID_HUB_DISP_PIX: expected = 3; break;
    case RP_CMD_ID_HUB_DISP_IMG: expected = 25; break;
    case RP_CMD_ID_HUB_SPK_PLY_TON: expected = 6; break;
    case RP_CMD_ID_HUB_DISP_TXT:
      if (size < 9 || memchr(param + 8, '\0', size - 8) == NULL)
        return RP_LINK_ERR_INVALID_SIZE;
      break;
    case RP_CMD_ID_HUB_DISP_TXT_SCR:
      if (size < 5 || memchr(param + 4, '\0', size - 4) == NULL)
        return RP_LINK_ERR_INVALID_SIZE;
      break;
    case RP_CMD_ID_HUB_LGT_ON_HSV:
      if (size != sizeof(pbio_color_hsv_t)) return RP_LINK_ERR_INVALID_SIZE;
      break;
    case RP_CMD_ID_HUB_LGT_ON_COL:
      if (size != sizeof(pbio_color_t)) return RP_LINK_ERR_INVALID_SIZE;
      break;
    default:
      return RP_LINK_ERR_UNSUPPORTED;
  }
  if (expected >= 0 && size != (size_t)expected) return RP_LINK_ERR_INVALID_SIZE;

  if (type >= RP_CMD_TYPE_COLOR && type <= RP_CMD_TYPE_US
      && GET_CMD_INDEX(cmd_id) != 0
      && fgDevices[port].config != type) {
    return RP_LINK_ERR_DEVICE_CONFLICT;
  }
  return RP_LINK_OK;
}

static void process_cmd(RasPikePort port, const int cmd_id,
                        char *param, size_t data_size)
{
  int validation = validate_command(port, cmd_id, param, data_size);
  if (validation != RP_LINK_OK) {
    ++fgLinkStats.invalid_commands;
    if (command_expects_ack(cmd_id)) send_ack(port, cmd_id, validation);
    return;
  }
  char cmd_type = GET_CMD_TYPE(cmd_id);
  switch(cmd_type) {
    case RP_CMD_TYPE_SYS:
      process_sys_cmd(port,cmd_id,param);
      break;
    case RP_CMD_TYPE_COLOR:
      process_color_sensor_cmd(port,cmd_id,param);
      break;
    case RP_CMD_TYPE_FORCE: 
      process_force_sensor_cmd(port,cmd_id,param);
      break;
    case RP_CMD_TYPE_MOTOR:
      process_motor_cmd(port,cmd_id,param);
      break;
    case RP_CMD_TYPE_US:
      process_ultrasonic_sensor_cmd(port,cmd_id,param);
      break;
    case RP_CMD_TYPE_HUB:
      process_hub_cmd(port,cmd_id,param);
      break;
    default:
      ++fgLinkStats.invalid_commands;
      break;
  }
}

static void notify_status(void)
{
  RPProtocolSpikeStatus snapshot;
  RPDevice devices[RP_MAX_DEVICES];

  /* Resolve delayed command selection and copy metadata only. Device reads can
     take far longer than a structure copy and must not inherit the real-time
     priority ceiling through APP_STATUS_MUTEX. */
  lock_status();
  for (int i = 0; i < RP_MAX_DEVICES; ++i) {
    const unsigned char next_cmd = fgDevices[i].next_cmd;
    if (next_cmd && next_cmd != fgCurrentStatus.ports[i].cmd) {
      fgCurrentStatus.ports[i].cmd = next_cmd;
    }
    fgDevices[i].next_cmd = 0;
  }
  snapshot = fgCurrentStatus;
  memcpy(devices, fgDevices, sizeof(devices));
  unlock_status();

  update_hub_status(&snapshot);
  update_port_devices(devices, RP_MAX_DEVICES, &snapshot);
  (void)raspike_send_data(RP_PORT_NONE, RP_CMD_ID_ALL_STATUS,
                          (const char *)&snapshot, sizeof(snapshot));
}



/*
 * Application Main Task
 */

/* ET Image*/
static uint8_t raspike2_image[5][5] = {
  {100,100,100,100,100},
  { 90,  0,  0, 90,  0},
  { 80, 80,  0, 80,  0},
  { 70,  0,  0, 70,  0},
  { 60, 60,  0, 60,  0}
};

/* Start Up Image*/
static uint8_t raspike2_startup_image[5][5] = {
  {  0,  0,  0,  0,  0},
  {  0,100,  0, 80,  0},
  {100,  0, 90,  0, 60},
  {  0,100,  0, 80,  0},
  {  0,  0,  0, 0,   0}
};




void main_task(intptr_t exinf)
{

  realtime_drive_cycle_counter_init();
  serial_opn_por(SIO_USB_PORTID);
  serial_ctl_por(SIO_USB_PORTID,0);
  // 1秒待たせる
  dly_tsk(1000000);

  //hub_display_image((uint8_t*)raspike2_image);

  /* command handling*/
  RasPikePort port;
  unsigned char cmd;
  unsigned char data_size;
  char buf[255];


  hub_display_image((uint8_t*)raspike2_startup_image);

  // Handshake: tolerate noise and read every field exactly.
  while (1) {
    unsigned char byte = 0;
    if (wait_read((char *)&byte, 1) < 0) continue;
    if (byte != RP_CMD_INIT) continue;
    if (wait_read((char *)&byte, 1) < 0) continue;
    if (byte == RP_CMD_INIT_MAGIC) break;
  }

  /* Initialize every Hub subsystem before advertising link readiness. */
  hub_imu_init();
  while (!hub_imu_is_ready()) {
    dly_tsk(10 * 1000);
  }
  fgRuntimeImuReadyLatched = true;

  if (begin_link_session(false) < 0) {
    hub_light_on_color(PBIO_COLOR_RED);
    hub_display_number(98);
    ext_tsk();
    return;
  }

  fgSafetyUsbTxRecoverySeen = pbdrv_usb_get_tx_recovery_count();
  fgSafetyUsbReinitSeen = pbdrv_usb_get_reinit_count();
  fgSafetyUsbRxOverflowSeen = pbdrv_usb_get_rx_overflow_count();
  fgSafetyUsbWasConnected = pbdrv_usb_is_connected();
  fgSafetyLastControlUpdateCount = fgDriveControllerUpdateCount;
  fgSafetyControlStaleCycles = 0u;
  fgSafetyTaskPending = false;
  pbdrv_watchdog_set_required_heartbeats(
      PBDRV_WATCHDOG_HEARTBEAT_USB | PBDRV_WATCHDOG_HEARTBEAT_SAFETY);
  sta_cyc(APP_SAFETY_CYC);

  /* Start slow complete status and compact 10 ms drive telemetry. */
  sta_cyc(APP_NOTIFY_CYC);
  sta_cyc(APP_DRIVE_TELEMETRY_CYC);

  /* display status :ready */
  hub_display_image((uint8_t*)raspike2_image);

  while (1) {
    memset(buf, 0, sizeof(buf));
    int receive_result = raspike_receive_data(
        buf, sizeof(buf), &port, &cmd, &data_size);
    if (receive_result == RP_LINK_RENEGOTIATED
        || receive_result == RP_REQUEST_REPLAYED) {
      continue;
    }
    if (receive_result < 0) {
      ++fgLinkStats.invalid_commands;
      dly_tsk(1000);
      continue;
    }
    process_cmd(port, cmd, buf, data_size);
  }



}

/* notification task*/
void notify_task(intptr_t exinf)
{
  notify_status();
  ext_tsk();
}


void safety_supervisor_cyclic_handler(intptr_t exinf)
{
  (void)exinf;
  if (fgSafetyTaskPending) return;
  fgSafetyTaskPending = true;
  if (act_tsk(APP_SAFETY_TASK) < 0) fgSafetyTaskPending = false;
}

void safety_supervisor_task(intptr_t exinf)
{
  (void)exinf;
  pbdrv_watchdog_report_heartbeat(PBDRV_WATCHDOG_HEARTBEAT_SAFETY);

  const uint32_t tx_recovery = pbdrv_usb_get_tx_recovery_count();
  const uint32_t reinit = pbdrv_usb_get_reinit_count();
  const uint32_t rx_overflow = pbdrv_usb_get_rx_overflow_count();
  const bool usb_connected = pbdrv_usb_is_connected();
  const bool usb_fault = tx_recovery != fgSafetyUsbTxRecoverySeen
      || reinit != fgSafetyUsbReinitSeen
      || rx_overflow != fgSafetyUsbRxOverflowSeen
      || (fgSafetyUsbWasConnected && !usb_connected);

  fgSafetyUsbTxRecoverySeen = tx_recovery;
  fgSafetyUsbReinitSeen = reinit;
  fgSafetyUsbRxOverflowSeen = rx_overflow;
  fgSafetyUsbWasConnected = usb_connected;

  const bool battery_critical = pbsys_status_test(
      PBIO_PYBRICKS_STATUS_BATTERY_LOW_VOLTAGE_SHUTDOWN);
  if (battery_critical) {
    fgSafetyInhibitReasons |= RP_SAFETY_INHIBIT_BATTERY;
  } else {
    fgSafetyInhibitReasons &= ~RP_SAFETY_INHIBIT_BATTERY;
  }

  bool control_fault = false;
  if (fgDriveRunning) {
    if (fgDriveControllerUpdateCount != fgSafetyLastControlUpdateCount) {
      fgSafetyLastControlUpdateCount = fgDriveControllerUpdateCount;
      fgSafetyControlStaleCycles = 0u;
    } else if (fgSafetyControlStaleCycles < UINT8_MAX) {
      ++fgSafetyControlStaleCycles;
      if (fgSafetyControlStaleCycles >= RP_SAFETY_CONTROL_STALE_CYCLES) {
        control_fault = true;
        fgSafetyInhibitReasons |= RP_SAFETY_INHIBIT_CONTROL;
      }
    }
  } else {
    fgSafetyControlStaleCycles = 0u;
    fgSafetyLastControlUpdateCount = fgDriveControllerUpdateCount;
  }

  if (usb_fault) {
    fgSafetyInhibitReasons |= RP_SAFETY_INHIBIT_USB;
  }

  if (usb_fault || battery_critical || control_fault) {
    ++fgSafetyGeneration;
    /* Low voltage uses coast to avoid regeneration. Link/control failures use
       brake for the shortest stopping distance. */
    hub_emergency_stop_all(!battery_critical);
    lock_status();
    fgDriveRunning = false;
    fgDriveMotorsStopped = true;
    fgDriveStatusFlags |= usb_fault
        ? RP_DRIVE_STATUS_TX_RECOVERED : RP_DRIVE_STATUS_WATCHDOG_STOP;
    if (usb_fault) {
      fgProtocolV2Active = false;
    }
    unlock_status();
  }

  uint32_t required = PBDRV_WATCHDOG_HEARTBEAT_USB
      | PBDRV_WATCHDOG_HEARTBEAT_SAFETY;
  if (fgDriveRunning || (fgSafetyInhibitReasons & RP_SAFETY_INHIBIT_CONTROL)) {
    required |= PBDRV_WATCHDOG_HEARTBEAT_CONTROL;
  }
  pbdrv_watchdog_set_required_heartbeats(required);

  fgSafetyTaskPending = false;
  ext_tsk();
}

void realtime_drive_cyclic_handler(intptr_t exinf)
{
  (void)exinf;
  if (fgDriveControlTaskPending) {
    ++fgRuntimeOverrunCount;
    ++fgDriveControllerDeadlineMissCount;
    fgDriveStatusFlags |= RP_DRIVE_STATUS_PERIOD_OVERRUN;
    return;
  }
  fgDriveControlTaskPending = true;
  if (act_tsk(APP_DRIVE_CONTROL_TASK) < 0) {
    fgDriveControlTaskPending = false;
    ++fgRuntimeOverrunCount;
    ++fgDriveControllerDeadlineMissCount;
  }
}

void critical_tx_task(intptr_t exinf)
{
  (void)exinf;
  for (;;) {
    RPCriticalTxSlot *slot = NULL;
    loc_cpu();
    if (fgCriticalTxCount > 0u) {
      RPCriticalTxSlot *candidate = &fgCriticalTxSlots[fgCriticalTxHead];
      if (candidate->state == RP_TX_SLOT_READY) {
        candidate->state = RP_TX_SLOT_SENDING;
        slot = candidate;
      }
    }
    if (!slot) {
      fgCriticalTxTaskScheduled = false;
      unl_cpu();
      break;
    }
    unl_cpu();

    (void)send_data_blocking((const char *)slot->data, slot->size);

    loc_cpu();
    fgCriticalTxCompletedTicket = slot->ticket;
    slot->ticket = 0u;
    slot->size = 0u;
    slot->state = RP_TX_SLOT_FREE;
    fgCriticalTxHead = (uint8_t)((fgCriticalTxHead + 1u) % RP_CRITICAL_TX_SLOT_COUNT);
    --fgCriticalTxCount;
    unl_cpu();
  }
  ext_tsk();
}

void realtime_drive_task(intptr_t exinf)
{
  (void)exinf;
  const uint32_t task_start_cycles = realtime_drive_now_cycles();
  const uint32_t now_us = realtime_drive_now_us();
  const uint32_t now_ms = now_us / 1000u;
  const RPPreparedDriveCommand prepared_command =
      realtime_drive_snapshot_command();
  lock_status();
  const uint32_t expected_period_us = fgDriveControlPeriodUs;
  const uint32_t period_us = fgDrivePreviousControlUs
      ? now_us - fgDrivePreviousControlUs : expected_period_us;
  fgDrivePreviousControlUs = now_us;
  const uint32_t period_limit_us = expected_period_us
      + expected_period_us / 2u;
  if (period_us > period_limit_us) {
    fgDriveStatusFlags |= RP_DRIVE_STATUS_PERIOD_OVERRUN;
    ++fgRuntimeOverrunCount;
    ++fgDriveControllerDeadlineMissCount;
  } else {
    fgDriveStatusFlags &= ~RP_DRIVE_STATUS_PERIOD_OVERRUN;
  }

  const RPRealtimeDriveCommand command = prepared_command.command;
  const bool command_fresh =
      now_ms - prepared_command.received_time_ms <= RP_DRIVE_WATCHDOG_MS;
  pup_motor_t *const left_motor = fgDriveLeftMotor;
  pup_motor_t *const right_motor = fgDriveRightMotor;
  pup_device_t *const color_sensor = fgDriveColorSensor;
  const bool configured = fgDriveConfigured;
  const bool running_requested = fgDriveRunning;
  const uint32_t safety_generation = fgSafetyGeneration;
  const bool recover_requested = fgRuntimeRecoverRequested;
  fgRuntimeRecoverRequested = false;
  unlock_status();

  int32_t left_count = 0;
  int32_t right_count = 0;
  int32_t left_speed = 0;
  int32_t right_speed = 0;
  const pbio_error_t left_state_error = left_motor
      ? pup_motor_get_state(left_motor, &left_count, &left_speed)
      : PBIO_ERROR_NO_DEV;
  const pbio_error_t right_state_error = right_motor
      ? pup_motor_get_state(right_motor, &right_count, &right_speed)
      : PBIO_ERROR_NO_DEV;
  int32_t reflection = -1;
  const pbio_error_t reflection_error = color_sensor
      ? pup_color_sensor_get_reflection_realtime(
          color_sensor, &reflection)
      : PBIO_ERROR_NO_DEV;
  const int error = reflection >= 0
      ? command.target_reflection - reflection : 0;
  float angular_velocity[3] = {0.0f, 0.0f, 0.0f};
  hub_imu_get_angular_velocity(angular_velocity);
  if (hub_imu_is_ready()) {
    fgRuntimeImuReadyLatched = true;
  }
  const int32_t gyro_rate_mdeg_s =
      (int32_t)(angular_velocity[2] * 1000.0f);

  const bool sensors_ok = configured
      && left_motor && right_motor && color_sensor
      && left_state_error == PBIO_SUCCESS
      && right_state_error == PBIO_SUCCESS
      && reflection_error == PBIO_SUCCESS
      && reflection >= 0;
  HubRuntimeInput runtime_input = {
    .configured = configured,
    .imu_ready = fgRuntimeImuReadyLatched,
    .calibration_ready = fgRuntimeImuReadyLatched,
    .preflight_ok = sensors_ok,
    .start_requested = running_requested,
    .stop_requested = !running_requested
        && (fgRuntimeState.state == HUB_RUNTIME_RUNNING
            || fgRuntimeState.state == HUB_RUNTIME_DEGRADED),
    .recover_requested = recover_requested,
    .command_fresh = command_fresh,
    .sensors_ok = sensors_ok,
    .period_ok = period_us <= period_limit_us,
    .motors_ok = true,
    .emergency_stop = false,
    .fatal_fault = false,
  };
  const uint32_t applied_fault_mask =
      hub_fault_injection_apply(&fgRuntimeFault, &runtime_input);
  HubRuntimeOutput runtime_output;
  (void)hub_runtime_step(&fgRuntimeParameters, &fgRuntimeState,
                         &runtime_input, &runtime_output);

  if (runtime_output.state == HUB_RUNTIME_SAFE_STOP
      || runtime_output.state == HUB_RUNTIME_FAULT) {
    realtime_drive_stop(runtime_output.reason == HUB_RUNTIME_REASON_COMMAND_TIMEOUT);
  }

  int left_pwm = 0;
  int right_pwm = 0;
  pbio_error_t left_power_error = PBIO_SUCCESS;
  pbio_error_t right_power_error = PBIO_SUCCESS;
  const bool running = running_requested && runtime_output.motors_allowed
      && fgSafetyGeneration == safety_generation
      && fgSafetyInhibitReasons == 0u;

  if (running) {
    const uint32_t controller_start_cycles = realtime_drive_now_cycles();
    bool controller_ok = true;
    if (command.mode == RP_DRIVE_MODE_HOST_CONTROL
        || command.mode == RP_DRIVE_MODE_MANUAL_PWM) {
      left_pwm = command.left_pwm;
      right_pwm = command.right_pwm;
      fgDriveLineControllerOutput = (LineTraceSpeedControllerOutput){
        .line_error = (int16_t)error,
        .steering_pwm = (int16_t)((right_pwm - left_pwm) / 2),
        .left_pwm = (float)left_pwm,
        .right_pwm = (float)right_pwm,
        .pwm_saturated = left_pwm <= -RP_DRIVE_PWM_LIMIT
            || left_pwm >= RP_DRIVE_PWM_LIMIT
            || right_pwm <= -RP_DRIVE_PWM_LIMIT
            || right_pwm >= RP_DRIVE_PWM_LIMIT,
      };
    } else if (command.mode == RP_DRIVE_MODE_YAW_RATE) {
      LineTraceSpeedControllerParameters parameters =
          prepared_command.controller_parameters;
      const float gyro_rate_rad_s =
          (angular_velocity[2] - prepared_command.gyro_bias_deg_s)
          * RP_DRIVE_YAW_RATE_SCALE * RP_DRIVE_DEG_TO_RAD;

      /* Preserve zero line error in yaw-only mode without rebuilding every
         other controller parameter in the 4 ms loop. */
      parameters.line.target_reflection = (int16_t)reflection;
      controller_ok = line_trace_speed_controller_update_with_gyro(
          &parameters, &fgDriveLineControllerState,
          prepared_command.target_speed_mm_s, 0.0f,
          prepared_command.curvature_per_mm,
          (int16_t)reflection, gyro_rate_rad_s,
          (float)left_speed, (float)right_speed,
          8.0f, prepared_command.dt_s,
          &fgDriveLineControllerOutput);
      if (controller_ok) {
        left_pwm = (int)fgDriveLineControllerOutput.left_pwm;
        right_pwm = (int)fgDriveLineControllerOutput.right_pwm;
      }
    } else if (command.mode == RP_DRIVE_MODE_HUB_CONTROL) {
      const float gyro_rate_rad_s =
          (angular_velocity[2] - prepared_command.gyro_bias_deg_s)
          * RP_DRIVE_YAW_RATE_SCALE * RP_DRIVE_DEG_TO_RAD;
      controller_ok = line_trace_speed_controller_update_with_gyro(
          &prepared_command.controller_parameters,
          &fgDriveLineControllerState,
          prepared_command.target_speed_mm_s, 0.0f,
          prepared_command.curvature_per_mm,
          (int16_t)reflection, gyro_rate_rad_s,
          (float)left_speed, (float)right_speed,
          8.0f, prepared_command.dt_s,
          &fgDriveLineControllerOutput);
      if (controller_ok) {
        left_pwm = (int)fgDriveLineControllerOutput.left_pwm;
        right_pwm = (int)fgDriveLineControllerOutput.right_pwm;
      }
    } else {
      controller_ok = false;
    }

    if (!controller_ok) {
      fgDriveStatusFlags |= RP_DRIVE_STATUS_SENSOR_ERROR;
      left_pwm = 0;
      right_pwm = 0;
    }
    left_pwm = realtime_drive_clamp(left_pwm, -RP_DRIVE_PWM_LIMIT, RP_DRIVE_PWM_LIMIT);
    right_pwm = realtime_drive_clamp(right_pwm, -RP_DRIVE_PWM_LIMIT, RP_DRIVE_PWM_LIMIT);
    fgDriveControllerExecutionUs = realtime_drive_cycles_to_us(
        realtime_drive_now_cycles() - controller_start_cycles);
    fgDriveControllerPeriodUs = period_us;
    ++fgDriveControllerUpdateCount;
    const uint32_t error_abs = (uint32_t)(error < 0 ? -error : error);
    fgDriveLineErrorSqSum += (uint64_t)error_abs * error_abs;
    fgDriveLineErrorAbsSum += error_abs;
    if (error_abs > fgDriveLineErrorAbsMax) {
      fgDriveLineErrorAbsMax = error_abs > UINT16_MAX
          ? UINT16_MAX : (uint16_t)error_abs;
    }
    if (fgDriveLineControllerOutput.pwm_saturated) {
      ++fgDrivePwmSaturationSamples;
    }
    left_power_error = pup_motor_set_power_realtime(left_motor, left_pwm);
    right_power_error = pup_motor_set_power_realtime(right_motor, right_pwm);
    const bool safety_changed = fgSafetyGeneration != safety_generation
        || fgSafetyInhibitReasons != 0u;
    if (safety_changed) {
      hub_emergency_stop_all(
          (fgSafetyInhibitReasons & RP_SAFETY_INHIBIT_BATTERY) == 0u);
      left_power_error = PBIO_ERROR_INVALID_OP;
      right_power_error = PBIO_ERROR_INVALID_OP;
    }
    if (left_power_error != PBIO_SUCCESS || right_power_error != PBIO_SUCCESS) {
      fgDriveStatusFlags |= RP_DRIVE_STATUS_SENSOR_ERROR;
      HubRuntimeInput motor_fault_input = runtime_input;
      motor_fault_input.motors_ok = false;
      (void)hub_runtime_step(&fgRuntimeParameters, &fgRuntimeState,
                             &motor_fault_input, &runtime_output);
      realtime_drive_stop(false);
      left_pwm = 0;
      right_pwm = 0;
      if (left_motor) pup_motor_brake(left_motor);
      if (right_motor) pup_motor_brake(right_motor);
      fgDriveMotorsStopped = true;
    } else {
      fgDriveStatusFlags &= ~RP_DRIVE_STATUS_SENSOR_ERROR;
      fgDriveMotorsStopped = false;
    }
  } else if (!fgDriveMotorsStopped) {
    if (left_motor) pup_motor_brake(left_motor);
    if (right_motor) pup_motor_brake(right_motor);
    fgDriveMotorsStopped = true;
  }


  RPRuntimeTelemetry runtime_telemetry = {0};
  runtime_telemetry.sequence = ++fgRuntimeTelemetrySequence;
  runtime_telemetry.hub_time_us = now_us;
  runtime_telemetry.sample_sequence = ++fgRuntimeSampleSequence;
  runtime_telemetry.control_period_us = period_us;
  runtime_telemetry.overrun_count = fgRuntimeOverrunCount;
  runtime_telemetry.transition_sequence = runtime_output.transition_sequence;
  runtime_telemetry.runtime_status_flags = runtime_output.status_flags;
  runtime_telemetry.active_fault_mask = applied_fault_mask;
  runtime_telemetry.left_count_deg = left_count;
  runtime_telemetry.right_count_deg = right_count;
  runtime_telemetry.left_speed_deg_s = left_speed;
  runtime_telemetry.right_speed_deg_s = right_speed;
  runtime_telemetry.gyro_rate_mdeg_s = gyro_rate_mdeg_s;
  runtime_telemetry.reflection = (int16_t)reflection;
  runtime_telemetry.line_error = (int16_t)error;
  runtime_telemetry.left_pwm = (int16_t)left_pwm;
  runtime_telemetry.right_pwm = (int16_t)right_pwm;
  runtime_telemetry.runtime_state = (uint8_t)runtime_output.state;
  runtime_telemetry.transition_reason = (uint8_t)runtime_output.reason;
  runtime_telemetry.active_mode = command.mode;
  runtime_telemetry.motors_allowed = runtime_output.motors_allowed ? 1u : 0u;
  runtime_telemetry.execution_time_us = realtime_drive_cycles_to_us(
      realtime_drive_now_cycles() - task_start_cycles);
  lock_status();
  if (fgSafetyGeneration != safety_generation || fgSafetyInhibitReasons != 0u) {
    fgDriveRunning = false;
    fgDriveMotorsStopped = true;
    runtime_telemetry.motors_allowed = 0u;
  }
  fgRuntimeTelemetrySnapshot = runtime_telemetry;
  fgDriveControlTaskPending = false;
  unlock_status();
  pbdrv_watchdog_report_heartbeat(PBDRV_WATCHDOG_HEARTBEAT_CONTROL);
  ext_tsk();
}

void drive_telemetry_task(intptr_t exinf)
{
  (void)exinf;
  RPDriveTelemetry telemetry = {0};
  pup_motor_t *motors[RP_DRIVE_TELEMETRY_MOTOR_SLOTS] = {0};
  pup_device_t *color = NULL;
  uint8_t motor_count = 0;

  lock_status();
  bool drive_active = fgDriveConfigured;
  if (!drive_active) {
    for (RasPikePort port = 0; port < RP_MAX_DEVICES; ++port) {
      if (fgDevices[port].config == RP_CMD_TYPE_MOTOR
          && fgDevices[port].device
          && fgCurrentStatus.ports[port].cmd == RP_CMD_ID_MOT_STU
          && motor_count < RP_DRIVE_TELEMETRY_MOTOR_SLOTS) {
        motors[motor_count] = (pup_motor_t *)fgDevices[port].device;
        telemetry.motor_port[motor_count] = port;
        ++motor_count;
      } else if (!color
                 && fgDevices[port].config == RP_CMD_TYPE_COLOR
                 && fgDevices[port].device
                 && fgCurrentStatus.ports[port].cmd == RP_CMD_ID_COL_REF) {
        color = (pup_device_t *)fgDevices[port].device;
        telemetry.color_port = port;
      }
    }
  }
  unlock_status();

  if (drive_active) {
    ext_tsk();
    return;
  }

  telemetry.sequence = ++fgDriveTelemetrySequence;
  if (telemetry.sequence == 0) telemetry.sequence = ++fgDriveTelemetrySequence;
  telemetry.hub_time_us = realtime_drive_now_us();
  for (uint8_t slot = 0; slot < motor_count; ++slot) {
    telemetry.motor_count_deg[slot] = pup_motor_get_count(motors[slot]);
    telemetry.motor_speed_deg_s[slot] = pup_motor_get_speed(motors[slot]);
    telemetry.motor_power[slot] = (int16_t)pup_motor_get_power(motors[slot]);
    telemetry.flags |= (uint8_t)(1u << slot);
  }
  if (color) {
    telemetry.reflection = (int16_t)pup_color_sensor_reflection(color);
    telemetry.flags |= RP_DRIVE_TELEMETRY_COLOR_VALID;
  }

  if (telemetry.flags != 0) {
    (void)raspike_send_data(RP_PORT_NONE, RP_CMD_ID_DRIVE_TELEMETRY,
                            (const char *)&telemetry, sizeof(telemetry));
  }
  ext_tsk();
}

void runtime_telemetry_task(intptr_t exinf)
{
  (void)exinf;
  RPRuntimeTelemetry telemetry;
  lock_status();
  telemetry = fgRuntimeTelemetrySnapshot;
  unlock_status();
  if (telemetry.sequence != 0u) {
    raspike_send_data(RP_PORT_NONE, RP_CMD_ID_RUNTIME_TELEMETRY,
                      (const char *)&telemetry, sizeof(telemetry));
  }
  ext_tsk();
}

/* soner sensor task*/
void soner_task(intptr_t exinf)
{
  update_ultrasonicsensor_port_devices(fgDevices,RP_MAX_DEVICES,&fgCurrentStatus);
  ext_tsk();

}
