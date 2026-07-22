/*
 * SPDX-License-Identifier: MIT
 *
 * API for the hub system.
 *
 * Copyright (c) 2023 Embedded and Real-Time Systems Laboratory,
 *            Graduate School of Information Science, Nagoya Univ., JAPAN
 */

#include <t_syslog.h>
#include <spike/hub/system.h>
#include <pbdrv/motor_driver.h>
#include <pbio/main.h>

/*
 * TODO: 
 *  - Check context
 *  - Close serial port
 */
void hub_system_shutdown(void) {
  void pybricks_c_pb_type_System_shutdown(void);
  pybricks_c_pb_type_System_shutdown();

  /* Never come back here */
  while(1);
}


void hub_emergency_stop_all(bool brake) {
  pbio_stop_all(true);
  if (brake) {
    pbdrv_motor_driver_emergency_brake_all();
  } else {
    pbdrv_motor_driver_emergency_coast_all();
  }
}

void hub_motor_emergency_disable_from_isr(void) {
  pbdrv_motor_driver_emergency_coast_all();
}
