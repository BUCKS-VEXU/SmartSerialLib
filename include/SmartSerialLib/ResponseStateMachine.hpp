/* Noah Klein */

#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "protocol/MessageStateMachine/StateMachine.hpp"
#include <iostream>

// Forward declare SmartSerial
class SmartSerial;

class ResponseStateMachine : public StateMachine {
  protected:
    void onChecksumFail() const override;
    void onMissingStartMarker(uint8_t byte) const override;
    void onMissingEndMarker(uint8_t byte) const override;
    void onMessageComplete() const override;

  private:
    SmartSerial *smartSerial; // Pointer to the SmartSerial instance

  public:
    ResponseStateMachine(size_t bufferSize, SmartSerial *serial) :
        StateMachine(bufferSize), smartSerial(serial) {}
};

#endif // STATE_MACHINE_HPP
