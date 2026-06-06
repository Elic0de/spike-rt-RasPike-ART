#include "comm/SerialProtocol.h"

#include <serial/serial.h>
#include <syssvc/serial.h>

namespace etrobo {
namespace comm {

void SerialProtocol::initialize()
{
  serial_opn_por(SIO_USB_PORTID);
  serial_ctl_por(SIO_USB_PORTID, 0);
}

void SerialProtocol::sendAck(uint8_t cmd, int32_t result) const
{
  int32_t payload[2] = { cmd, result };
  sendFrame(APP_MSG_ACK, payload, sizeof(payload));
}

void SerialProtocol::sendStatus(const LineStatus& status) const
{
  sendFrame(APP_MSG_STATUS, &status, sizeof(status));
}

bool SerialProtocol::receive(uint8_t& cmd, uint8_t& size, char* payload,
                             std::size_t payloadSize, bool wait)
{
  uint8_t header[4];

  for (;;) {
    if (readExact(reinterpret_cast<char*>(header), 1, wait) != 1) {
      return false;
    }
    if (header[0] == APP_FRAME_START) {
      break;
    }
    if (!wait) {
      return false;
    }
  }

  if (readExact(reinterpret_cast<char*>(header), 3, wait) != 3) {
    return false;
  }

  cmd = header[0];
  size = header[1];

  if (size > payloadSize) {
    drainPayload(size);
    return false;
  }
  if (size > 0 && readExact(payload, size, wait) != size) {
    return false;
  }

  return true;
}

void SerialProtocol::sendFrame(uint8_t msg, const void* payload, uint8_t size) const
{
  uint8_t header[4] = { APP_FRAME_START, msg, size, 0 };

  serial_wri_dat(SIO_USB_PORTID, reinterpret_cast<const char*>(header), sizeof(header));
  if (payload != nullptr && size > 0) {
    serial_wri_dat(SIO_USB_PORTID, reinterpret_cast<const char*>(payload), size);
  }
}

int SerialProtocol::readSome(char* buf, std::size_t size, bool wait)
{
  for (;;) {
    int len = serial_rea_dat(SIO_USB_PORTID, buf, size);
    if (len > 0 || !wait) {
      return len;
    }
    dly_tsk(1000);
  }
}

int SerialProtocol::readExact(char* buf, std::size_t size, bool wait)
{
  std::size_t done = 0;

  while (done < size) {
    int len = readSome(buf + done, size - done, wait);
    if (len <= 0) {
      return done;
    }
    done += len;
  }

  return done;
}

void SerialProtocol::drainPayload(uint8_t size)
{
  char discard[32];

  while (size > 0) {
    std::size_t chunk = (size > sizeof(discard)) ? sizeof(discard) : size;
    readExact(discard, chunk, true);
    size -= chunk;
  }
}

}  // namespace comm
}  // namespace etrobo
