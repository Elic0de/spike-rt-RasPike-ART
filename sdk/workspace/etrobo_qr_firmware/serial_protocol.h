#ifndef ETROBO_QR_FIRMWARE_SERIAL_PROTOCOL_H
#define ETROBO_QR_FIRMWARE_SERIAL_PROTOCOL_H

#include "protocol.h"

#include <stddef.h>
#include <stdint.h>

void serial_protocol_init(void);
void serial_protocol_send_ack(uint8_t cmd, int32_t result);
void serial_protocol_send_status(const LineStatus *status);
int serial_protocol_receive(uint8_t *cmd, uint8_t *size, char *payload, size_t payload_size);

#endif
