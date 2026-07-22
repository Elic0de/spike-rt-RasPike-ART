#ifndef _RASPIKE_PROTOCOL_H
#define _RASPIKE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include "raspike_protocol_com.h"

#define RP_CMD_INIT (0xAE)
#define RP_CMD_START (0xEA)

#define RP_CMD_INIT_MAGIC (0xCE)

/* RasPike protocol v2: session-aware, sequenced, CRC-protected frames. */
#define RP_V2_START                    (0xEB)
#define RP_V2_VERSION                  (2u)
#define RP_V2_HEADER_SIZE              (14u) /* includes start byte */
#define RP_V2_CRC_SIZE                 (2u)
#define RP_V2_FRAME_OVERHEAD           (RP_V2_HEADER_SIZE + RP_V2_CRC_SIZE)
#define RP_V2_MAX_PAYLOAD              (240u)

#define RP_V2_OFFSET_VERSION           (1u)
#define RP_V2_OFFSET_FLAGS             (2u)
#define RP_V2_OFFSET_CMD               (3u)
#define RP_V2_OFFSET_PORT              (4u)
#define RP_V2_OFFSET_SIZE              (5u)
#define RP_V2_OFFSET_SEQUENCE          (6u)
#define RP_V2_OFFSET_SESSION           (10u)
#define RP_V2_OFFSET_PAYLOAD           (14u)

#define RP_V2_FLAG_ACK                 (1u << 0)
#define RP_V2_FLAG_ERROR               (1u << 1)
#define RP_V2_FLAG_HIGH_PRIORITY       (1u << 2)
#define RP_V2_FLAG_TELEMETRY           (1u << 3)

#define RP_V2_CAP_FRAMING              (1u << 0)
#define RP_V2_CAP_SEQUENCE             (1u << 1)
#define RP_V2_CAP_SESSION              (1u << 2)
#define RP_V2_CAP_CRC16                (1u << 3)
#define RP_V2_CAP_IDEMPOTENT_CONFIG    (1u << 4)
#define RP_V2_CAP_SOFT_RESET           (1u << 5)
#define RP_V2_CAP_BATCHED_TX           (1u << 6)
#define RP_V2_CAP_LINK_STATS           (1u << 7)
#define RP_V2_CAPABILITIES             (RP_V2_CAP_FRAMING | RP_V2_CAP_SEQUENCE | \
                                        RP_V2_CAP_SESSION | RP_V2_CAP_CRC16 | \
                                        RP_V2_CAP_IDEMPOTENT_CONFIG | \
                                        RP_V2_CAP_SOFT_RESET | \
                                        RP_V2_CAP_BATCHED_TX | RP_V2_CAP_LINK_STATS)

#define RP_CMD_ID_LINK_HELLO           (MAKE_CMD(RP_CMD_TYPE_SYS,0x08))
#define RP_CMD_ID_LINK_PING            (MAKE_CMD(RP_CMD_TYPE_SYS,0x09))
#define RP_CMD_ID_LINK_STATS           (MAKE_CMD(RP_CMD_TYPE_SYS,0x0A))
#define RP_CMD_ID_DRIVE_TELEMETRY      (MAKE_CMD(RP_CMD_TYPE_SYS,0x0C))
#define RP_CMD_ID_RUNTIME_FAULT         (MAKE_CMD(RP_CMD_TYPE_SYS,0x0D))
#define RP_CMD_ID_RUNTIME_TELEMETRY     (MAKE_CMD(RP_CMD_TYPE_SYS,0x0E))

#define RP_V2_HELLO_PAYLOAD_SIZE       (12u)
#define RP_V2_STATS_PAYLOAD_SIZE       (32u)

#define RP_LINK_OK                     (0)
#define RP_LINK_ERR_INVALID_PORT       (-1)
#define RP_LINK_ERR_INVALID_SIZE       (-2)
#define RP_LINK_ERR_CRC                (-3)
#define RP_LINK_ERR_SESSION            (-4)
#define RP_LINK_ERR_UNSUPPORTED        (-5)
#define RP_LINK_ERR_DEVICE_CONFLICT    (-6)
#define RP_LINK_ERR_TIMEOUT            (-7)

static inline uint16_t rp_v2_get_u16(const unsigned char *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rp_v2_get_u32(const unsigned char *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void rp_v2_put_u16(unsigned char *p, uint16_t value)
{
  p[0] = (unsigned char)(value & 0xffu);
  p[1] = (unsigned char)((value >> 8) & 0xffu);
}

static inline void rp_v2_put_u32(unsigned char *p, uint32_t value)
{
  p[0] = (unsigned char)(value & 0xffu);
  p[1] = (unsigned char)((value >> 8) & 0xffu);
  p[2] = (unsigned char)((value >> 16) & 0xffu);
  p[3] = (unsigned char)((value >> 24) & 0xffu);
}

static inline uint16_t rp_v2_crc16(const unsigned char *data, size_t size)
{
  uint16_t crc = 0xffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}


#define GET_CMD_TYPE(cmd) ((cmd)>>5)
#define GET_CMD_INDEX(cmd) ((cmd)&0x1f)
#define MAKE_CMD(type,id) ((type)<<5 | id)

#define RP_CMD_TYPE_SYS (0x0)
#define RP_CMD_TYPE_COLOR (0x1)
#define RP_CMD_TYPE_FORCE (0x2)
#define RP_CMD_TYPE_MOTOR (0x3)
#define RP_CMD_TYPE_US (0x4) // Ultrasonic Sensor
#define RP_CMD_TYPE_HUB (0x5) 



// PORT 
#define RP_PORT_NONE (255)

// SYSTEM CMD
#define RP_CMD_ID_ALL_STATUS (MAKE_CMD(RP_CMD_TYPE_SYS,0x1)) // 0x01
#define RP_CMD_ID_ACK (MAKE_CMD(RP_CMD_TYPE_SYS,0x2)) // 0x02
#define RP_CMD_ID_SHT_DWN (MAKE_CMD(RP_CMD_TYPE_SYS,0x3))
#define RP_CMD_ID_SOFT_RST (MAKE_CMD(RP_CMD_TYPE_SYS,0x4)) // 0x04
#define RP_CMD_ID_UPDATE_MODE (MAKE_CMD(RP_CMD_TYPE_SYS,0x0B)) // 0x0B
#define RP_SOFT_RESET_MAGIC (0x52535431u) // ASCII "RST1"
#define RP_UPDATE_MODE_MAGIC (0x55504431u) // ASCII "UPD1"




/* ET Robocon real-time drive extension. */
#define RP_CMD_ID_DRIVE_CONFIG    (MAKE_CMD(RP_CMD_TYPE_SYS,0x5)) /* 0x05 */
#define RP_CMD_ID_DRIVE_COMMAND   (MAKE_CMD(RP_CMD_TYPE_SYS,0x6)) /* 0x06 */

#define RP_DRIVE_MODE_SAFE_STOP   0
#define RP_DRIVE_MODE_PI_CONTROL  1
#define RP_DRIVE_MODE_HOST_CONTROL RP_DRIVE_MODE_PI_CONTROL
#define RP_DRIVE_MODE_HUB_CONTROL 2
#define RP_DRIVE_MODE_MANUAL_PWM  3
#define RP_DRIVE_MODE_YAW_RATE   4

#define RP_DRIVE_FLAG_ARM   (1u << 0)
#define RP_DRIVE_FLAG_START (1u << 1)
#define RP_DRIVE_FLAG_STOP  (1u << 2)

#define RP_DRIVE_STATUS_ARMED          (1u << 0)
#define RP_DRIVE_STATUS_RUNNING        (1u << 1)
#define RP_DRIVE_STATUS_WATCHDOG_STOP  (1u << 2)
#define RP_DRIVE_STATUS_PERIOD_OVERRUN (1u << 3)
#define RP_DRIVE_STATUS_SENSOR_ERROR   (1u << 4)
#define RP_DRIVE_STATUS_TX_RECOVERED    (1u << 5)

typedef struct __attribute__((packed)) {
  uint32_t sequence;
  uint32_t sent_time_us;
  uint8_t mode;
  uint8_t flags;
  int16_t target_speed_mm_s;
  int16_t target_reflection;
  int16_t kp_milli;
  int16_t ki_milli;
  int16_t kd_milli;
  int16_t left_pwm;
  int16_t right_pwm;
  uint16_t control_period_us;
  uint16_t reserved0;
  uint32_t source_telemetry_sequence;
  int32_t curvature_per_mm_e9;
} RPRealtimeDriveCommand;


#define RP_DRIVE_TELEMETRY_MOTOR_SLOTS 2u
#define RP_DRIVE_TELEMETRY_MOTOR0_VALID (1u << 0)
#define RP_DRIVE_TELEMETRY_MOTOR1_VALID (1u << 1)
#define RP_DRIVE_TELEMETRY_COLOR_VALID  (1u << 2)

typedef struct __attribute__((packed)) {
  uint32_t fault_mask;
  uint32_t duration_cycles;
} RPRuntimeFaultCommand;

typedef struct __attribute__((packed)) {
  uint32_t sequence;
  uint32_t hub_time_us;
  uint32_t sample_sequence;
  uint32_t control_period_us;
  uint32_t execution_time_us;
  uint32_t overrun_count;
  uint32_t transition_sequence;
  uint32_t runtime_status_flags;
  uint32_t active_fault_mask;
  int32_t left_count_deg;
  int32_t right_count_deg;
  int32_t left_speed_deg_s;
  int32_t right_speed_deg_s;
  int32_t gyro_rate_mdeg_s;
  int16_t reflection;
  int16_t line_error;
  int16_t left_pwm;
  int16_t right_pwm;
  uint8_t runtime_state;
  uint8_t transition_reason;
  uint8_t active_mode;
  uint8_t motors_allowed;
} RPRuntimeTelemetry;

typedef struct __attribute__((packed)) {
  uint32_t sequence;
  uint32_t hub_time_us;
  int32_t motor_count_deg[RP_DRIVE_TELEMETRY_MOTOR_SLOTS];
  int32_t motor_speed_deg_s[RP_DRIVE_TELEMETRY_MOTOR_SLOTS];
  int16_t motor_power[RP_DRIVE_TELEMETRY_MOTOR_SLOTS];
  int16_t reflection;
  uint8_t motor_port[RP_DRIVE_TELEMETRY_MOTOR_SLOTS];
  uint8_t color_port;
  uint8_t flags;
} RPDriveTelemetry;

// COLOR SENSOR CMD
#define RP_CMD_ID_COL_CFG (MAKE_CMD(RP_CMD_TYPE_COLOR,0x0)) // 0x20
#define RP_CMD_ID_COL_RGB (MAKE_CMD(RP_CMD_TYPE_COLOR,0x1))// 0x21
#define RP_CMD_ID_COL_COL (MAKE_CMD(RP_CMD_TYPE_COLOR,0x2)) // 0x22
#define RP_CMD_ID_COL_COL_SUR_OFF (MAKE_CMD(RP_CMD_TYPE_COLOR,0x3)) // 0x23
#define RP_CMD_ID_COL_HSV (MAKE_CMD(RP_CMD_TYPE_COLOR,0x4)) // 0x24
#define RP_CMD_ID_COL_HSV_SUR_OFF (MAKE_CMD(RP_CMD_TYPE_COLOR,0x5)) // 0x25
#define RP_CMD_ID_COL_REF (MAKE_CMD(RP_CMD_TYPE_COLOR,0x6))// 0x26
#define RP_CMD_ID_COL_AMB (MAKE_CMD(RP_CMD_TYPE_COLOR,0x7)) // 0x27 
#define RP_CMD_ID_COL_LIGHT_SET (MAKE_CMD(RP_CMD_TYPE_COLOR,0x8)) // 0x28
#define RP_CMD_ID_COL_LIGHT_ON (MAKE_CMD(RP_CMD_TYPE_COLOR,0x9)) // 0x29
#define RP_CMD_ID_COL_LIGHT_OFF (MAKE_CMD(RP_CMD_TYPE_COLOR,0xa)) // 0x2a
#define RP_CMD_ID_COL_DETECTABLE_COL (MAKE_CMD(RP_CMD_TYPE_COLOR,0xb)) // 0x2b

// FORCE SENSOR CMD
#define RP_CMD_ID_FRC_CFG (MAKE_CMD(RP_CMD_TYPE_FORCE,0x0)) // 0x40
#define RP_CMD_ID_FRC_FRC (MAKE_CMD(RP_CMD_TYPE_FORCE,0x1)) // 0x41
#define RP_CMD_ID_FRC_DST (MAKE_CMD(RP_CMD_TYPE_FORCE,0x2)) // 0x42                                                                
#define RP_CMD_ID_FRC_PRS (MAKE_CMD(RP_CMD_TYPE_FORCE,0x3)) // 0x43
#define RP_CMD_ID_FRC_TCH (MAKE_CMD(RP_CMD_TYPE_FORCE,0x4)) // 0x44

/* Force sensor status protocol                                                                                                                                                                       
   port->data[0]-data[3] : Force (float)                                                                                                                                                              
   port->data[4]-data[7] : Distance (float)                                                                                                                                                           
   port->data[8] : Is Touched (bool 1byte)                                                                                                                                                            
*/
#define RP_FORCESENSOR_INDEX_FRC (0)
#define RP_FORCESENSOR_INDEX_DST (4)
#define RP_FORCESENSOR_INDEX_TCH (8)

// MOTOR
#define RP_CMD_ID_MOT_CFG (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x0)) // 0x60
#define RP_CMD_ID_MOT_STU (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x1)) // 0x61
#define RP_CMD_ID_MOT_RST (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x2)) // 0x62
#define RP_CMD_ID_MOT_SPD (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x3)) // 0x63
#define RP_CMD_ID_MOT_POW (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x4)) // 0x64
#define RP_CMD_ID_MOT_STP (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x5)) // 0x65
#define RP_CMD_ID_MOT_STP_BRK (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x6)) // 0x66
#define RP_CMD_ID_MOT_STP_HLD (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x7)) // 0x67
#define RP_CMD_ID_MOT_SET_DTY (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x8)) // 0x68
#define RP_CMD_ID_MOT_RST_DTY (MAKE_CMD(RP_CMD_TYPE_MOTOR,0x9)) // 0x69

/* Motor Set up protocol                                                                                                                                                                              
   data[0]-data[3] : positive directoon                                                                                                                                                               
   data[4] : reset_count                                                                                                                                                                              
*/
#define RP_MOTOR_STU_INDEX_DIRECTION (0)
#define RP_MOTOR_STU_INDEX_RESETCOUNT (4)

/* Motor status status protocol                                                                                                                                                                              
   port->data[0]-data[3] : count(int32_t)                                                                                                                                                             
   port->data[4]-data[7] : speed(int32_t)                                                                                                                                                             
   port->data[8]-data[9] : power(int16_t) this is not same as original api                                                                                                                            
   port->data[10]        : is stalled                                                                                                                                                                 
*/
#define RP_MOTOR_INDEX_COUNT (0)
#define RP_MOTOR_INDEX_SPEED (4)
#define RP_MOTOR_INDEX_POWER (8)
#define RP_MOTOR_INDEX_ISSTALLED (10)

// ULTRASONIC CMD
#define RP_CMD_ID_US_CFG (MAKE_CMD(RP_CMD_TYPE_US,0x0)) // 0x80
#define RP_CMD_ID_US_DST (MAKE_CMD(RP_CMD_TYPE_US,0x1)) // 0x81  Not Used
#define RP_CMD_ID_US_PRC (MAKE_CMD(RP_CMD_TYPE_US,0x2)) // 0x82  Not Used
#define RP_CMD_ID_US_LGT_SET (MAKE_CMD(RP_CMD_TYPE_US,0x3)) // 0x83
#define RP_CMD_ID_US_LGT_ON (MAKE_CMD(RP_CMD_TYPE_US,0x4)) // 0x84
#define RP_CMD_ID_US_LGT_OFF (MAKE_CMD(RP_CMD_TYPE_US,0x5)) // 0x85

/* Ultrasonic sensor cmd protocol
  LGT_SET
  int32_t x 4 : bv1-bv4
*/
#define RP_US_LGT_SET_INDEX_BV1 (0)
#define RP_US_LGT_SET_INDEX_BV2 (4)
#define RP_US_LGT_SET_INDEX_BV3 (8)
#define RP_US_LGT_SET_INDEX_BV4 (12)


/* Ultrasonic sensor status protocol
  port->data[0]-data[3] : distance(int32_t)
  port->data[4]         : presence(bool)
*/
#define RP_US_INDEX_DISTANCE (0)
#define RP_US_INDEX_PRESENCE (4)

/* HUB CMD*/
#define RP_CMD_ID_HUB_DISP_ORI (MAKE_CMD(RP_CMD_TYPE_HUB,0x1)) // 0xa1
#define RP_CMD_ID_HUB_DISP_OFF (MAKE_CMD(RP_CMD_TYPE_HUB,0x2)) // 0xa2
#define RP_CMD_ID_HUB_DISP_PIX (MAKE_CMD(RP_CMD_TYPE_HUB,0x3)) // 0xa3
#define RP_CMD_ID_HUB_DISP_IMG (MAKE_CMD(RP_CMD_TYPE_HUB,0x4)) // 0xa4
#define RP_CMD_ID_HUB_DISP_NUM (MAKE_CMD(RP_CMD_TYPE_HUB,0x5)) // 0xa5
#define RP_CMD_ID_HUB_DISP_CHR (MAKE_CMD(RP_CMD_TYPE_HUB,0x6)) // 0xa6
#define RP_CMD_ID_HUB_DISP_TXT (MAKE_CMD(RP_CMD_TYPE_HUB,0x7)) // 0xa7
#define RP_CMD_ID_HUB_DISP_TXT_SCR (MAKE_CMD(RP_CMD_TYPE_HUB,0x8)) // 0xa8
#define RP_CMD_ID_HUB_LGT_ON_HSV (MAKE_CMD(RP_CMD_TYPE_HUB,0x0a)) // 0xaa
#define RP_CMD_ID_HUB_LGT_ON_COL (MAKE_CMD(RP_CMD_TYPE_HUB,0x0b)) // 0xab
#define RP_CMD_ID_HUB_LGT_OFF (MAKE_CMD(RP_CMD_TYPE_HUB,0x0c)) // 0xac
#define RP_CMD_ID_HUB_SPK_SET_VOL (MAKE_CMD(RP_CMD_TYPE_HUB,0x11)) // 0xb1
#define RP_CMD_ID_HUB_SPK_PLY_TON (MAKE_CMD(RP_CMD_TYPE_HUB,0x12)) // 0xb2
#define RP_CMD_ID_HUB_SPK_STP (MAKE_CMD(RP_CMD_TYPE_HUB,0x13)) // 0xb3
#define RP_CMD_ID_HUB_IMU_SET_TLT (MAKE_CMD(RP_CMD_TYPE_HUB,0x14)) // 0xb4
#define RP_CMD_ID_HUB_IMU_RST_HDG (MAKE_CMD(RP_CMD_TYPE_HUB,0x15)) // 0xb5


/* Hub cmd protocol
  RP_CMD_ID_HUB_DISP_PIX
  data[0] : row (uint8_t) 
  data[1] : column (uint8_t) 
  data[2] : brightness (uint8_t)
*/
#define RP_HUB_DISP_PIX_INDEX_ROW (0)
#define RP_HUB_DISP_PIX_INDEX_COL (1)
#define RP_HUB_DISP_PIX_INDEX_BRT (2)

/* Hub cmd protocol
  RP_CMD_ID_HUB_DISP_TXT
  data[0]-[3] : on (uint32_t) 
  data[4]-[7]: off (uint32_t)
  data[8]- : text(null terminate) 
*/
#define RP_HUB_DISP_TXT_INDEX_ON (0)
#define RP_HUB_DISP_TXT_INDEX_OFF (4)
#define RP_HUB_DISP_TXT_INDEX_TXT (8)

/* Hub cmd protocol
  RP_CMD_ID_HUB_DISP_TXT_SCR
  data[0]-[3] : delay (uint32_t) 
  data[4]- text(null terminate) 
*/
#define RP_HUB_DISP_TXT_SCR_INDEX_DLY (0)
#define RP_HUB_DISP_TXT_SCR_INDEX_TXT (4)

/* Hub cmd protocol
  RP_CMD_ID_HUB_SPK_PLY_TON
  data[0]-[3] : duration (int32_t)
  data[4]-[5] : frequency (uint16_t)
*/
#define RP_HUB_SPK_PLY_TON_INDEX_DUR (0)
#define RP_HUB_SPK_PLY_TON_INDEX_FRQ (4)


/* In SPIKE-RT, ID port is defined as 'A' to 'F'. 
   RasPike treat them as 0 to 5 
   PORT_TO_RASPIKE and PORT_FROM_RASPIKE are used for this conversion.*/
#define PORT_TO_RASPIKE(port) ((port)-'A')   
#define PORT_FROM_RASPIKE(raspike_port) ((raspike_port)+'A')   

typedef unsigned char RasPikePort;

typedef struct {
    RasPikePort port;
    unsigned char cmd;
    unsigned char _paddig[2];
    unsigned char data[12];
} RPProtocolPortStatus;


typedef struct {
    uint16_t voltage;
    uint16_t current;
    float acceleration[3];
    float angular_velocity[3];
    float heading;
    uint16_t is_ready;
    uint16_t is_statinary;
    uint32_t button;
    RPProtocolPortStatus ports[6];
} RPProtocolSpikeStatus;

#define RP_PROTOCOL_BUFMAX (256)


// For set_speed,set_power
typedef struct {
  int val;
} RPProtocolParamMotorValue;



#endif // _RASPIKE_PROTOCOL_H