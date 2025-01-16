/* Noah Klein */

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "SmartSerialLib/SmartSerial.hpp"

inline void ResponseStateMachine::reset() {
    currentState = STATE_IDLE;
    UUID = 0;
    bufferIndex = 0;
    payloadLength = 0;
    checksum = 0;
    calculatedChecksum = 0;
}

inline bool ResponseStateMachine::isIdle() {
    return currentState == STATE_IDLE;
}

State ResponseStateMachine::getCurrentState() {
    return currentState;
}

// TODO the state machine doesn't report any errors to SmartSerial right now

void ResponseStateMachine::loop(uint8_t byte) {
    switch (currentState) {
    case STATE_IDLE:
        if (byte == START_MARKER) {
            currentState = STATE_UUID;
        } else {
            std::cout << "Expected start marker, " << std::hex << START_MARKER
                      << ", but received " << std::hex << byte << "\n";
        }
        break;

    case STATE_UUID:
        UUID = byte;
        calculatedChecksum = byte;
        currentState = STATE_COMMAND;
        break;

    case STATE_COMMAND:
        commandID = byte; // Store command ID
        calculatedChecksum += byte;
        currentState = STATE_LENGTH;
        break;

    case STATE_LENGTH:
        payloadLength = byte;
        calculatedChecksum += byte;
        currentState = (payloadLength > 0) ? STATE_PAYLOAD : STATE_CHECKSUM;
        break;

    case STATE_PAYLOAD:
        buffer[bufferIndex++] = byte;
        calculatedChecksum += byte;

        if (bufferIndex == payloadLength) {
            currentState = STATE_CHECKSUM;
        }
        break;

    case STATE_CHECKSUM:
        checksum = byte;
        if (checksum != calculatedChecksum) {
            std::cout << "Checksum mismatch\n Expected:" << calculatedChecksum
                      << "\n Received: " << checksum << "\n";
            this->reset();
            return;
        }

        currentState = STATE_COMPLETE;
        break;

    case STATE_COMPLETE:
        if (byte != END_MARKER) {
            std::cout << "Expected end marker, " << std::hex << END_MARKER
                      << ", but received " << std::hex << byte << "\n";
            this->reset();
            return;
        }

        auto payload = std::make_shared<std::vector<uint8_t>>(
            buffer, buffer + payloadLength);
        SerialResponse response = {
            UUID, commandID, payloadLength, payload, checksum};

        this->smartSerial->addResponse(response);
        this->reset();

        break;
    }
}
