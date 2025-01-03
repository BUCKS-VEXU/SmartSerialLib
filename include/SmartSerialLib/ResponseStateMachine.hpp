/* Noah Klein */

#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <cstddef>
#include <cstdint>

// Forward declaration of SmartSerial
class SmartSerial;

// Enum for states
enum State {
  STATE_IDLE,
  STATE_UUID,
  STATE_LENGTH,
  STATE_COMMAND,
  STATE_PAYLOAD,
  STATE_CHECKSUM,
  STATE_COMPLETE
};

class ResponseStateMachine {
private:
  State currentState = STATE_IDLE;
  uint8_t buffer[256]; // Buffer for incoming messages
  size_t bufferIndex = 0;

  uint8_t UUID = 0;
  uint8_t payloadLength = 0;
  uint8_t commandID = 0;
  uint8_t checksum = 0;
  uint8_t calculatedChecksum = 0;

  SmartSerial *smartSerial; // Pointer to the SmartSerial instance

public:
  ResponseStateMachine(SmartSerial *serial) : smartSerial(serial) {}
  inline void reset();
  inline bool isIdle();
  void loop(uint8_t byte);
  State getCurrentState();
  // Add methods as needed
};

#endif // STATE_MACHINE_HPP
