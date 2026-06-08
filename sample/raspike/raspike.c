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

#include <spike/pup/motor.h>
#include <spike/pup/colorsensor.h>
#include <spike/pup/forcesensor.h>
#include <spike/pup/ultrasonicsensor.h>


#include "raspike.h"
#define RP_DEFINE_CMD_SIZE
#include "raspike_protocol_com.h"
//#include "tPutLogTarget_tecsgen.h"

#define RP_MAX_DEVICES (6)

#ifndef RASPIKE_MEASURE_USB_LOOP
#define RASPIKE_MEASURE_USB_LOOP 1
#endif

#if RASPIKE_MEASURE_USB_LOOP
#ifndef RASPIKE_MEASURE_MAX_SAMPLES
#define RASPIKE_MEASURE_MAX_SAMPLES 256
#endif
#define RASPIKE_MEASURE_TARGET_US 10000UL
#define RASPIKE_MEASURE_TOLERANCE_US 1000UL
#define RASPIKE_MEASURE_HIST_BIN_US 500UL
#define RASPIKE_MEASURE_HIST_BINS 101U

typedef struct {
  unsigned long timestamp_us;
  unsigned long dt_us;
  unsigned long body_us;
  long deadline_lag_us;
  int port;
  int cmd;
  int value;
} RPMeasureSample;

typedef struct {
  uint32_t count;
  uint32_t min_dt_us;
  uint32_t max_dt_us;
  uint32_t out_of_range;
  uint32_t dropped;
  uint32_t first_timestamp_us;
  uint32_t last_timestamp_us;
  uint64_t sum_dt_us;
  uint32_t hist[RASPIKE_MEASURE_HIST_BINS];
} RPMeasureStats;

static RPMeasureSample fgMotorPowerSamples[RASPIKE_MEASURE_MAX_SAMPLES];
static RPMeasureSample fgNotifySamples[RASPIKE_MEASURE_MAX_SAMPLES];
static unsigned int fgMotorPowerSampleCount = 0;
static unsigned int fgNotifySampleCount = 0;
static unsigned long fgLastMotorPowerUs[RP_MAX_DEVICES] = {0};
static unsigned long fgLastNotifyUs = 0;
static unsigned long fgNextNotifyDeadlineUs = 0;
static int fgMeasureFlushed = 0;
static RPMeasureStats fgMotorPowerStats = {0};
static RPMeasureStats fgNotifyStats = {0};

static int raspike_send_data(RasPikePort port , int msg_id, const char *buf, size_t size);

static unsigned long measure_now_us(void)
{
  SYSTIM now = 0;
  ER ercd = get_tim(&now);
  return ercd == E_OK ? (unsigned long)now : 0UL;
}

static void measure_stats_update(RPMeasureStats *stats, unsigned long timestamp_us, unsigned long dt_us)
{
  if (dt_us == 0) return;
  if (stats->count == 0) {
    stats->min_dt_us = (uint32_t)dt_us;
    stats->first_timestamp_us = (uint32_t)timestamp_us;
  }
  if (dt_us < stats->min_dt_us) stats->min_dt_us = (uint32_t)dt_us;
  if (dt_us > stats->max_dt_us) stats->max_dt_us = (uint32_t)dt_us;
  if (dt_us < RASPIKE_MEASURE_TARGET_US - RASPIKE_MEASURE_TOLERANCE_US ||
      dt_us > RASPIKE_MEASURE_TARGET_US + RASPIKE_MEASURE_TOLERANCE_US) {
    ++stats->out_of_range;
  }

  unsigned int bin = dt_us / RASPIKE_MEASURE_HIST_BIN_US;
  if (bin >= RASPIKE_MEASURE_HIST_BINS) {
    bin = RASPIKE_MEASURE_HIST_BINS - 1;
  }
  ++stats->hist[bin];
  ++stats->count;
  stats->sum_dt_us += dt_us;
  stats->last_timestamp_us = (uint32_t)timestamp_us;
}

static int measure_is_out_of_range(unsigned long dt_us)
{
  return dt_us != 0 &&
         (dt_us < RASPIKE_MEASURE_TARGET_US - RASPIKE_MEASURE_TOLERANCE_US ||
          dt_us > RASPIKE_MEASURE_TARGET_US + RASPIKE_MEASURE_TOLERANCE_US);
}

static uint32_t measure_stats_percentile(const RPMeasureStats *stats, uint32_t pct)
{
  if (stats->count == 0) return 0;
  uint32_t target = (stats->count * pct + 99U) / 100U;
  uint32_t cumulative = 0;
  for (unsigned int i = 0; i < RASPIKE_MEASURE_HIST_BINS; ++i) {
    cumulative += stats->hist[i];
    if (cumulative >= target) {
      return (uint32_t)(i * RASPIKE_MEASURE_HIST_BIN_US);
    }
  }
  return (uint32_t)((RASPIKE_MEASURE_HIST_BINS - 1U) * RASPIKE_MEASURE_HIST_BIN_US);
}

static void measure_send_rows(int source_id, const RPMeasureSample *samples, unsigned int count)
{
  for (unsigned int i = 0; i < count; ++i) {
    const RPMeasureSample *s = samples + i;
    RPProtocolMeasureRow row = {
      source_id,
      (int32_t)i,
      (int32_t)s->timestamp_us,
      (int32_t)s->dt_us,
      (int32_t)s->body_us,
      (int32_t)s->deadline_lag_us,
      (int32_t)s->port,
      (int32_t)s->cmd,
      (int32_t)s->value,
    };
    raspike_send_data(RP_PORT_NONE, RP_CMD_ID_MEASURE_ROW, (const char *)&row, sizeof(row));
  }
}

static void measure_send_row(int source_id, int seq, const RPMeasureSample *s)
{
  RPProtocolMeasureRow row = {
    source_id,
    (int32_t)seq,
    (int32_t)s->timestamp_us,
    (int32_t)s->dt_us,
    (int32_t)s->body_us,
    (int32_t)s->deadline_lag_us,
    (int32_t)s->port,
    (int32_t)s->cmd,
    (int32_t)s->value,
  };
  raspike_send_data(RP_PORT_NONE, RP_CMD_ID_MEASURE_ROW, (const char *)&row, sizeof(row));
}

static void measure_send_stats(int source_id, const RPMeasureStats *stats)
{
  uint32_t avg = stats->count == 0 ? 0U : (uint32_t)(stats->sum_dt_us / stats->count);
  RPProtocolMeasureStats row = {
    source_id,
    (int32_t)stats->count,
    (int32_t)stats->min_dt_us,
    (int32_t)stats->max_dt_us,
    (int32_t)avg,
    (int32_t)measure_stats_percentile(stats, 95),
    (int32_t)measure_stats_percentile(stats, 99),
    (int32_t)stats->out_of_range,
    (int32_t)stats->dropped,
    (int32_t)stats->first_timestamp_us,
    (int32_t)stats->last_timestamp_us,
    (int32_t)(stats->sum_dt_us & 0xffffffffULL),
    (int32_t)(stats->sum_dt_us >> 32),
  };
  raspike_send_data(RP_PORT_NONE, RP_CMD_ID_MEASURE_STATS, (const char *)&row, sizeof(row));
}

static void measure_flush(void)
{
  if (fgMeasureFlushed) return;
  fgMeasureFlushed = 1;

  measure_send_rows(1, fgMotorPowerSamples, fgMotorPowerSampleCount);
  measure_send_rows(2, fgNotifySamples, fgNotifySampleCount);
  measure_send_stats(1, &fgMotorPowerStats);
  measure_send_stats(2, &fgNotifyStats);
}

static void measure_motor_power_rx(RasPikePort port, int cmd, int value, unsigned long body_us)
{
  if (port < 0 || port >= RP_MAX_DEVICES) return;
  unsigned long now_us = measure_now_us();
  unsigned long dt_us = fgLastMotorPowerUs[port] == 0 ? 0UL : now_us - fgLastMotorPowerUs[port];
  measure_stats_update(&fgMotorPowerStats, now_us, dt_us);
  fgLastMotorPowerUs[port] = now_us;

  RPMeasureSample sample = {
    now_us,
    dt_us,
    body_us,
    0,
    port,
    cmd,
    value,
  };

  if (fgMotorPowerSampleCount < RASPIKE_MEASURE_MAX_SAMPLES) {
    fgMotorPowerSamples[fgMotorPowerSampleCount] = sample;
    ++fgMotorPowerSampleCount;
  } else {
    ++fgMotorPowerStats.dropped;
  }

  if (measure_is_out_of_range(dt_us)) {
    measure_send_row(3, (int)fgMotorPowerStats.out_of_range, &sample);
  }
}

static void measure_notify_task(unsigned long start_us, unsigned long end_us)
{
  if (fgNextNotifyDeadlineUs == 0) {
    fgNextNotifyDeadlineUs = start_us;
  }
  unsigned long dt_us = fgLastNotifyUs == 0 ? 0UL : start_us - fgLastNotifyUs;
  measure_stats_update(&fgNotifyStats, start_us, dt_us);
  fgLastNotifyUs = start_us;

  RPMeasureSample sample = {
    start_us,
    dt_us,
    end_us >= start_us ? end_us - start_us : 0UL,
    start_us >= fgNextNotifyDeadlineUs
        ? (long)(start_us - fgNextNotifyDeadlineUs)
        : -(long)(fgNextNotifyDeadlineUs - start_us),
    -1,
    -1,
    0,
  };
  fgNextNotifyDeadlineUs += 10000UL;

  if (fgNotifySampleCount < RASPIKE_MEASURE_MAX_SAMPLES) {
    fgNotifySamples[fgNotifySampleCount] = sample;
    ++fgNotifySampleCount;
  } else {
    ++fgNotifyStats.dropped;
  }

  if (measure_is_out_of_range(dt_us)) {
    measure_send_row(4, (int)fgNotifyStats.out_of_range, &sample);
  }
}
#endif

#define RP_ASSERT(cond,val) rp_assert(cond,val)

static void rp_assert(int cond, int val)
{
  if ( !cond ) { 
    hub_light_on_color(PBIO_COLOR_RED); 
    hub_display_number(val);
  }
}

static RPProtocolSpikeStatus fgCurrentStatus = {0};

typedef struct {
  void *device;
  unsigned char config;
  unsigned char next_cmd;
  unsigned char sub_cmd; // for motor reset
} RPDevice;

static RPDevice fgDevices[RP_MAX_DEVICES] = {0};

static void lock_status(void)
{
  loc_mtx(APP_STATUS_MUTEX);
}

static void unlock_status(void)
{
  unl_mtx(APP_STATUS_MUTEX);
}


static int send_data(const char* buf, size_t size)
{
  ER ercd;

  ercd = serial_wri_dat(SIO_USB_PORTID,buf,size);

  if ( ercd == E_OK ) return size;

  return 0;

}

static int wait_read(char* buf, size_t size)
{
  size_t left = size;
  char *p = buf;
  do {
    int len = serial_rea_dat(SIO_USB_PORTID, p, 1);
    left -= len;
    p+=len;
  } while ( left > 0 );

  return size;

}


//----- Protocol Send
static int raspike_send_data(RasPikePort port , int msg_id, const char *buf, size_t size)
{
  char header[4];
  header[0] = RP_CMD_START;
  header[1] = msg_id;
  header[2] = size;
  header[3] = port;
  loc_mtx(APP_SEND_MUTEX);
  send_data(header,sizeof(header));
  if ( buf > 0 && size > 0) {
    send_data(buf,size);
  }
  unl_mtx(APP_SEND_MUTEX);
  return size;
}

static int raspike_receive_data(char *buf, size_t size , RasPikePort *port, unsigned char *cmd, unsigned char *data_size ) 
{
  char header[4] = {0};

  // First , seach START byte
  int i = 0;
  while (1) {
    int len = wait_read(header,1);
    if ( (len == 1) && (header[0] == RP_CMD_START) ) {
      break;  
    }
//    hub_display_number(i);
    i++;
  }

  // Second, read remaining header
  wait_read(header,3);
  *cmd = header[0];
  *data_size = header[1];
  *port = header[2];

  // hub_display_number(*cmd);
  if ( (*data_size > 0) && (*data_size <= size) && ( buf )) {
    wait_read(buf,*data_size);
  }
  return *data_size;
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
  // Not lock port config because this may be called only initializing
  RP_ASSERT(port >=0 && port <= 5, 10);
  fgDevices[port].config = type;
  fgDevices[port].device = device;

}

static void update_device_cmd(RasPikePort port, const int cmd_id, char *data, size_t data_size)
{
  /* Set next command to device. This is applied by notification task*/
  fgDevices[port].next_cmd = cmd_id;
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

static void send_ack(RasPikePort port, const char cmd_id, int32_t data)
{  
  int32_t send_data[2] = { (int32_t)cmd_id,data };
  raspike_send_data(port,RP_CMD_ID_ACK,(char*)send_data,sizeof(send_data));
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

void update_port_device_motor(unsigned char cmd_id,unsigned char sub_cmd,pup_motor_t *dev,RPProtocolPortStatus *status)
{
  // setupする前にgetするとSPIKEが死んでしまう
  if ( cmd_id != RP_CMD_ID_MOT_STU ) 
    return;

/* ここでリセットのAckを返すと、リセットしてすぐにカウントを取った時に前の情報が残っている可能性があるので、ackの位置を変更する*/
  if ( sub_cmd == RP_CMD_ID_MOT_RST ) {
//      int32_t success = 0;
/* リセットだけは行う*/
      pbio_error_t err = pup_motor_reset_count(dev);
//      if ( err == PBIO_SUCCESS ) {
//        success = 1;
        // lock status and modify count of the motor to ensure reset is executed
//      } 
//      send_ack(status->port,RP_CMD_ID_MOT_RST,success);
  }

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
  lock_status();
  *(int32_t*)(status->data+RP_US_INDEX_DISTANCE) = distance;
  //*(bool *)(status->data+RP_US_INDEX_PRESENCE) = presence;
  unlock_status();
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
  unsigned char sub_cmd = device->sub_cmd;

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
      update_port_device_motor(cmd_id,sub_cmd,(pup_motor_t*)dev,status);
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
  lock_status();
  for ( i = 0 ; i < num; i++ ) {
    void *dev = devices[i].device;
    if ( dev ) {
//      hub_display_number(i);
      update_port_device(devices+i,status->ports+i);
    }
  }
  unlock_status();
}

void update_ultrasonicsensor_port_devices(RPDevice *devices, int num, RPProtocolSpikeStatus *status)
{
  int i;
  for ( i = 0 ; i < num; i++ ) {
    void *dev = devices[i].device;
    if ( dev && devices[i].config == RP_CMD_TYPE_US ) {
//      hub_display_number(i);
      update_port_device_ultrasonicsensor((status->ports+i)->cmd, dev, status->ports+i);
    }
  }
}




static void process_sys_cmd(RasPikePort port, const int cmd_id, char *param)
{
  switch (cmd_id ) {
    case RP_CMD_ID_SHT_DWN:
#if RASPIKE_MEASURE_USB_LOOP
      measure_flush();
#endif
      hub_system_shutdown();
      // Not reached
      break;
    case RP_CMD_ID_RESTART:
#if RASPIKE_MEASURE_USB_LOOP
      measure_flush();
#endif
      pbdrv_reset(PBDRV_RESET_ACTION_RESET);
      // Not reached
      break;
    case RP_CMD_ID_MEASURE_FLUSH:
#if RASPIKE_MEASURE_USB_LOOP
      measure_flush();
#endif
      break;
    default:
      break;
  }


}

static void process_color_sensor_cmd(RasPikePort port, int cmd_id, char *param) 
{
  switch (cmd_id ) {
    case RP_CMD_ID_COL_CFG:
      RP_ASSERT(fgDevices[port].device==0,20);
      lock_status();
      void *device = pup_color_sensor_get_device(PORT_FROM_RASPIKE(port));
      {
        int32_t success = 0;
        if ( device ) {
          success = 1;
          update_port_config(port,RP_CMD_TYPE_COLOR,device);
        } 
        send_ack(port,cmd_id,success);
      }
      unlock_status();

      break;
    // following commands do not need immediate ack. 
    case RP_CMD_ID_COL_RGB:
    case RP_CMD_ID_COL_COL:
    case RP_CMD_ID_COL_COL_SUR_OFF:
    case RP_CMD_ID_COL_HSV:
    case RP_CMD_ID_COL_HSV_SUR_OFF:
    case RP_CMD_ID_COL_REF:
    case RP_CMD_ID_COL_AMB:
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_COLOR, 21);
      // If cmd is not changed, just return
      if ( fgCurrentStatus.ports[port].cmd == (char) cmd_id ) {
         return;
      }
      update_device_cmd(port,cmd_id,0,0);
      break;
    case RP_CMD_ID_COL_LIGHT_SET:
      {
        RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_COLOR, 22);
        int32_t *bv;
        bv = (int32_t*)param;
        pbio_error_t err = pup_color_sensor_light_set(fgDevices[port].device,*bv,*(bv+1),*(bv+2));
        send_ack(port,cmd_id,(int32_t)err);

      }
    default:
//      RP_ASSERT(false,22);
      break;
  }
}

static void process_force_sensor_cmd(RasPikePort port, const int cmd_id, const char *param) 
{
  switch (cmd_id ) {
    case RP_CMD_ID_FRC_CFG:
      RP_ASSERT(fgDevices[port].device==0,40);
      lock_status();
      void *device = pup_force_sensor_get_device(PORT_FROM_RASPIKE(port));
      {
        int32_t success = 0;
        if ( device ) {
          success = 1;
          update_port_config(port,RP_CMD_TYPE_FORCE,device);
        } 
        // Send port status immediately as acknowledgement
        send_ack(port,cmd_id,success);
      }
      unlock_status();

      break;    
#if 0
// No use these command (always return as status)
    case RP_CMD_ID_FRC_FRC:
    case RP_CMD_ID_FRC_DST:
    case RP_CMD_ID_FRC_PRS:
    case RP_CMD_ID_FRC_TCH:
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_FORCE, 41);
      // If cmd is not changed, just return
      if ( fgCurrentStatus.ports[port].cmd == (char) cmd_id ) {
         return;
      }
      update_device_cmd(port,cmd_id,0,0);
#endif 
    default:
      RP_ASSERT(false,42);
  }



}

static void process_motor_cmd(RasPikePort port, const int cmd_id, const char *param)
{
  switch (cmd_id ) {
    case RP_CMD_ID_MOT_CFG:
      RP_ASSERT(fgDevices[port].device==0,60);
      lock_status();
      void *device = pup_motor_get_device(PORT_FROM_RASPIKE(port));
      {
        int32_t success = 0;
        if ( device ) {
          success = 1;
          update_port_config(port,RP_CMD_TYPE_MOTOR,device);
        } 
        // Send port status immediately as acknowledgement
        send_ack(port,cmd_id,success);        
      }
      unlock_status();
      break;    
    case RP_CMD_ID_MOT_STU:
    // Set up the motor
    { 
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 61);
      pup_direction_t direction = *(pup_direction_t*)(param+RP_MOTOR_STU_INDEX_DIRECTION);
      bool reset = *(bool*)(param+RP_MOTOR_STU_INDEX_RESETCOUNT);
      int32_t success = 0;
      pbio_error_t err = pup_motor_setup(fgDevices[port].device,direction,reset);
      if ( err == PBIO_SUCCESS ) {
        success = 1;
        update_device_cmd(port,cmd_id,0,0);
        // if reset is requested, lock the status and modify count of the motor to ensure reset is executed
        if ( reset ) {
          lock_status();
          *(int32_t *)&fgCurrentStatus.ports[port].data[RP_MOTOR_INDEX_COUNT] = 0;
          send_ack(port,cmd_id,success);
          unlock_status();
        } else {
          // no need to lock. just send ack.
          send_ack(port,cmd_id,success);
        }
      } else {
        // no need to lock. just send ack.
        send_ack(port,cmd_id,success);
      }
    }
    break;
    case RP_CMD_ID_MOT_RST:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 62);
      /* Original
      int32_t success = 0;
      pbio_error_t err = pup_motor_reset_count(fgDevices[port].device);
      if ( err == PBIO_SUCCESS ) {
        success = 1;
        // lock status and modify count of the motor to ensure reset is executed

        lock_status();
        *(int32_t *)&fgCurrentStatus.ports[port].data[RP_MOTOR_INDEX_COUNT] = 0;
        send_ack(port,cmd_id,success);
        unlock_status();
      } else {
        // no need to lock. just send ack.
        send_ack(port,cmd_id,success);
      }
      */
      /* delayed reset version to ensure counter is reset*/
      fgDevices[port].sub_cmd = RP_CMD_ID_MOT_RST;
      // Send port status immediately as acknowledgement
    }
    break;
    case RP_CMD_ID_MOT_SPD:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 63);
      RPProtocolParamMotorValue *pm = (RPProtocolParamMotorValue *)param;
      // do not check error
      pup_motor_set_speed(fgDevices[port].device,pm->val);

    }
    break;
    case RP_CMD_ID_MOT_POW:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 64);
      RPProtocolParamMotorValue *pm = (RPProtocolParamMotorValue *)param;
      // do not check error
#if RASPIKE_MEASURE_USB_LOOP
      unsigned long measure_start_us = measure_now_us();
#endif
      pup_motor_set_power(fgDevices[port].device,pm->val);
#if RASPIKE_MEASURE_USB_LOOP
      unsigned long measure_end_us = measure_now_us();
      measure_motor_power_rx(port, cmd_id, pm->val,
                             measure_end_us >= measure_start_us
                                 ? measure_end_us - measure_start_us
                                 : 0UL);
#endif
    }
    break;
    case RP_CMD_ID_MOT_STP:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 65);
      pbio_error_t err = pup_motor_stop(fgDevices[port].device);
      RP_ASSERT(err == PBIO_SUCCESS,66);
      send_ack(port,cmd_id,1);
    }
    break;  
    case RP_CMD_ID_MOT_STP_BRK:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 67);
      pbio_error_t err = pup_motor_brake(fgDevices[port].device);
      RP_ASSERT(err == PBIO_SUCCESS,68);
      send_ack(port,cmd_id,1);
     
    }
    break;
    case RP_CMD_ID_MOT_STP_HLD:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 69);
      pbio_error_t err = pup_motor_hold(fgDevices[port].device);
      RP_ASSERT(err == PBIO_SUCCESS,70);
      send_ack(port,cmd_id,1);
    }
    break;
    case RP_CMD_ID_MOT_SET_DTY:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 64);
      RPProtocolParamMotorValue *pm = (RPProtocolParamMotorValue *)param;
      pbio_error_t err = pup_motor_set_duty_limit(fgDevices[port].device,pm->val);
      send_ack(port,cmd_id,err);
    }
    break;
    case RP_CMD_ID_MOT_RST_DTY:
    {
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_MOTOR, 64);
      RPProtocolParamMotorValue *pm = (RPProtocolParamMotorValue *)param;
      pup_motor_set_duty_limit(fgDevices[port].device,pm->val);
      // RestoreLimit is void
      send_ack(port,cmd_id,1);
    }
    break;

    default:
      RP_ASSERT(false,79);
  }

}

static void process_ultrasonic_sensor_cmd(RasPikePort port, const int cmd_id, const char *param) 
{
  switch (cmd_id) {
    case RP_CMD_ID_US_CFG:
      RP_ASSERT(fgDevices[port].device==0,80);
      lock_status();
      void *device = pup_ultrasonic_sensor_get_device(PORT_FROM_RASPIKE(port));
      {
        int32_t success = 0;
        if ( device ) {
          success = 1;
          update_port_config(port,RP_CMD_TYPE_US,device);
          sta_cyc(APP_SONER_CYC);
        } 
        // Send port status immediately as acknowledgement
        send_ack(port,cmd_id,success);
      }
      unlock_status();
      break;   
    case RP_CMD_ID_US_LGT_SET:
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_US, 81);
      RP_ASSERT(param != 0, 85);
      {
        int32_t bv1 = *(int32_t*)(param+RP_US_LGT_SET_INDEX_BV1);
        int32_t bv2 = *(int32_t*)(param+RP_US_LGT_SET_INDEX_BV2);
        int32_t bv3 = *(int32_t*)(param+RP_US_LGT_SET_INDEX_BV3);
        int32_t bv4 = *(int32_t*)(param+RP_US_LGT_SET_INDEX_BV4);
        // As ack is not needed, error check is eliminated 
        pup_ultrasonic_sensor_light_set(fgDevices[port].device,bv1,bv2,bv3,bv4);
      }
      break;
    case RP_CMD_ID_US_LGT_ON:
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_US, 82);
      pup_ultrasonic_sensor_light_on(fgDevices[port].device);
      break;
    case RP_CMD_ID_US_LGT_OFF:
      RP_ASSERT(fgDevices[port].config == RP_CMD_TYPE_US, 83);
      pup_ultrasonic_sensor_light_off(fgDevices[port].device);
      break;
  }
}

static void process_hub_cmd(RasPikePort port, const int cmd_id, const char *param)
{
  switch(cmd_id) {
    case RP_CMD_ID_HUB_DISP_ORI:
      {
        uint8_t ori = *(uint8_t*)param;
        hub_display_orientation(ori);
      }
    break;

    case RP_CMD_ID_HUB_DISP_OFF:
      hub_display_off();
      break;
    
    case RP_CMD_ID_HUB_DISP_PIX:
      {
        uint8_t row = *(uint8_t*)(param+RP_HUB_DISP_PIX_INDEX_ROW);
        uint8_t col = *(uint8_t*)(param+RP_HUB_DISP_PIX_INDEX_COL);
        uint8_t bright = *(uint8_t*)(param+RP_HUB_DISP_PIX_INDEX_BRT);
        hub_display_pixel(row,col,bright);
      }
      break;

    case RP_CMD_ID_HUB_DISP_IMG:
      {
        hub_display_image((uint8_t*)param);
      }
      break;

    case RP_CMD_ID_HUB_DISP_NUM:
      hub_display_number(*(int8_t*)param);
      break;

    case RP_CMD_ID_HUB_DISP_CHR:
      hub_display_char((*(char *)param));
      break;

    case RP_CMD_ID_HUB_DISP_TXT:
      {
        uint32_t on_ms = *(uint32_t*)(param + RP_HUB_DISP_TXT_INDEX_ON);
        uint32_t off_ms = *(uint32_t*)(param + RP_HUB_DISP_TXT_INDEX_OFF);
        hub_display_text(param+RP_HUB_DISP_TXT_INDEX_TXT,on_ms,off_ms);
      }
      break;
    
    case RP_CMD_ID_HUB_DISP_TXT_SCR:
      {
        uint32_t delay = *(uint32_t*)(param+RP_HUB_DISP_TXT_SCR_INDEX_DLY);
        hub_display_text_scroll(param+RP_HUB_DISP_TXT_SCR_INDEX_TXT,delay);
      }
      break;

     case RP_CMD_ID_HUB_LGT_ON_HSV:
      hub_light_on_hsv((pbio_color_hsv_t*)param);
      break;

    case RP_CMD_ID_HUB_LGT_ON_COL:
      hub_light_on_color(*(pbio_color_t*)param);
      break;

    case RP_CMD_ID_HUB_LGT_OFF:
      hub_light_off();
      break; 

    case RP_CMD_ID_HUB_SPK_SET_VOL:
      hub_speaker_set_volume(*(uint8_t*)param);
      break;

    case RP_CMD_ID_HUB_SPK_PLY_TON:
      {
       int32_t duration = *(int32_t*)(param+RP_HUB_SPK_PLY_TON_INDEX_DUR);
       uint16_t frequency = *(uint16_t*)(param+RP_HUB_SPK_PLY_TON_INDEX_FRQ); 
        hub_speaker_play_tone(frequency,duration);
      }
      break;
    case RP_CMD_ID_HUB_SPK_STP:
      hub_speaker_stop();
      break;

    default:
      RP_ASSERT(fgDevices[port].device==0,99);
      break;


    }


}

static void process_cmd(RasPikePort port, const int cmd_id, char *param)
{
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
      RP_ASSERT(false,3);
      break;
  }
}

/* コマンドによっては、ステータスを返した後にackを返す必要があるものがある
(モータのリセット)。そのようなものはここで返す*/
void send_delayed_ack(RPDevice *devices, int num)
{
  for (int i = 0; i < num; i++ ) {
    RPDevice *dev = devices+i;
    if (dev->sub_cmd == RP_CMD_ID_MOT_RST) {
      //　成功しか返せない
      send_ack(i,RP_CMD_ID_MOT_RST,1);
      // clear
      dev->sub_cmd = 0;
    }
  }
}

static void notify_status(void)
{
  update_hub_status(&fgCurrentStatus);
  update_port_devices(fgDevices,RP_MAX_DEVICES,&fgCurrentStatus);
  raspike_send_data(RP_PORT_NONE,RP_CMD_ID_ALL_STATUS,(const char*)&fgCurrentStatus,sizeof(fgCurrentStatus));
  send_delayed_ack(fgDevices,RP_MAX_DEVICES);

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
  hub_imu_init();

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

  // Hand Shake
  while (1) {
//    wait_read(buf,1);
    serial_rea_dat(SIO_USB_PORTID, buf, 1); 
    //hub_display_number(buf[0]);
    if ( buf[0] == RP_CMD_INIT ) {
      serial_rea_dat(SIO_USB_PORTID, buf, 1); 
      if ( buf[0] == RP_CMD_INIT_MAGIC ) {
        break;
      }
        //hub_display_number(buf[0]);
    }
  }
  
  // send INIT and version

  // version is given by -D option
#ifndef SPIKE_EXPECTED_VERSION_MAJOR
#define SPIKE_EXPECTED_VERSION_MAJOR 0
#endif
#ifndef SPIKE_EXPECTED_VERSION_MINOR
#define SPIKE_EXPECTED_VERSION_MINOR 0
#endif
#ifndef SPIKE_EXPECTED_VERSION_PATCH
#define SPIKE_EXPECTED_VERSION_PATCH 0
#endif

  buf[0] = RP_CMD_INIT;
  buf[1] = RP_CMD_INIT_MAGIC;
  buf[2] = SPIKE_EXPECTED_VERSION_MAJOR;
  buf[3] = SPIKE_EXPECTED_VERSION_MINOR;
  buf[4] = SPIKE_EXPECTED_VERSION_PATCH;

  serial_wri_dat(SIO_USB_PORTID,buf,5);

  memset(fgDevices,0,sizeof(fgDevices));

  for (int i=0 ; i < RP_MAX_DEVICES; i++ ){
    fgCurrentStatus.ports[i].port = i;
  }

  /* start notification */
  sta_cyc(APP_NOTIFY_CYC);

  /* display status :ready */
  hub_display_image((uint8_t*)raspike2_image);

  while(1) {
    raspike_receive_data(buf,sizeof(buf),&port,&cmd,&data_size);
//    hub_display_number(cmd);
    process_cmd(port,cmd,buf);
  }



}

/* notification task*/
void notify_task(intptr_t exinf)
{
#if RASPIKE_MEASURE_USB_LOOP
  unsigned long measure_start_us = measure_now_us();
#endif
  notify_status();
#if RASPIKE_MEASURE_USB_LOOP
  unsigned long measure_end_us = measure_now_us();
  measure_notify_task(measure_start_us, measure_end_us);
#endif
  ext_tsk();
}

/* soner sensor task*/
void soner_task(intptr_t exinf)
{
  update_ultrasonicsensor_port_devices(fgDevices,RP_MAX_DEVICES,&fgCurrentStatus);
  ext_tsk();

}
