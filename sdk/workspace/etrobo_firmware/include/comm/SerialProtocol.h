#ifndef ETROBO_FIRMWARE_COMM_SERIAL_PROTOCOL_H
#define ETROBO_FIRMWARE_COMM_SERIAL_PROTOCOL_H

#include "protocol.h"

#include <cstddef>
#include <cstdint>

namespace etrobo {
namespace comm {

class SerialProtocol {
 public:
  void initialize();
  void sendAck(uint8_t cmd, int32_t result) const;
  void sendStatus(const LineStatus& status) const;
  bool receive(uint8_t& cmd, uint8_t& size, char* payload, std::size_t payloadSize,
               bool wait);

 private:
  void sendFrame(uint8_t msg, const void* payload, uint8_t size) const;
  int readSome(char* buf, std::size_t size, bool wait);
  int readExact(char* buf, std::size_t size, bool wait);
  void drainPayload(uint8_t size);
};

}  // namespace comm
}  // namespace etrobo

#endif
