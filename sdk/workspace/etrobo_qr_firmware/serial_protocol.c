#include "serial_protocol.h"

#include "app.h"

#include <syssvc/serial.h>
#include <serial/serial.h>

static void send_frame(uint8_t msg, const void *payload, uint8_t size)
{
  uint8_t header[4] = { APP_FRAME_START, msg, size, 0 };

  serial_wri_dat(SIO_USB_PORTID, (const char *)header, sizeof(header));
  if (payload && size > 0) {
    serial_wri_dat(SIO_USB_PORTID, (const char *)payload, size);
  }
}

static int read_exact(char *buf, size_t size)
{
  size_t done = 0;

  while (done < size) {
    int len = serial_rea_dat(SIO_USB_PORTID, buf + done, size - done);
    if (len > 0) {
      done += len;
    } else {
      dly_tsk(1000);
    }
  }

  return done;
}

static void drain_payload(uint8_t size)
{
  char discard[32];

  while (size > 0) {
    size_t chunk = (size > sizeof(discard)) ? sizeof(discard) : size;
    read_exact(discard, chunk);
    size -= chunk;
  }
}

void serial_protocol_init(void)
{
  serial_opn_por(SIO_USB_PORTID);
  serial_ctl_por(SIO_USB_PORTID, 0);
}

void serial_protocol_send_ack(uint8_t cmd, int32_t result)
{
  int32_t payload[2] = { cmd, result };
  send_frame(APP_MSG_ACK, payload, sizeof(payload));
}

void serial_protocol_send_status(const LineStatus *status)
{
  send_frame(APP_MSG_STATUS, status, sizeof(*status));
}

int serial_protocol_receive(uint8_t *cmd, uint8_t *size, char *payload, size_t payload_size)
{
  uint8_t header[4];

  for (;;) {
    read_exact((char *)header, 1);
    if (header[0] == APP_FRAME_START) {
      break;
    }
  }

  read_exact((char *)header, 3);
  *cmd = header[0];
  *size = header[1];

  if (*size > payload_size) {
    drain_payload(*size);
    return 0;
  }
  if (*size > 0) {
    read_exact(payload, *size);
  }

  return 1;
}
