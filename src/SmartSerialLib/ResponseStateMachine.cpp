/* Noah Klein */

#include "SmartSerialLib/ResponseStateMachine.hpp"
#include "SmartSerialLib/SmartSerial.hpp"
#include "SmartSerialLib/protocol/ProtocolDefinitions.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

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

void ResponseStateMachine::loop(uint8_t byte) {
    this->smartSerial->totalBytesRead++;

    switch (currentState) {
    case STATE_IDLE:
        if (byte == START_MARKER) {
            currentState = STATE_UUID;
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
        currentState = STATE_PAYLOAD;
        break;

    case STATE_PAYLOAD:
        if (bufferIndex == payloadLength) {
            currentState = STATE_CHECKSUM;
            return;
        }
        buffer[bufferIndex++] = byte;
        calculatedChecksum += byte;
        break;

    case STATE_CHECKSUM:
        checksum = byte;
        if (checksum != calculatedChecksum) {
            std::cout << "Checksum mismatch\n Expected:" << calculatedChecksum
                      << "\n Received: " << checksum << "\n";
            this->reset();
            break;
        }

        currentState = STATE_COMPLETE;
        break;

    case STATE_COMPLETE:
        if (byte != START_MARKER) {
            std::cout << "Expected end marker, " << START_MARKER
                      << ", but received " << byte << "\n";
            this->reset();
            return;
        }

        auto payload = std::make_shared<std::vector<uint8_t>>(
            buffer, buffer + payloadLength);
        SerialResponse response = {
            UUID, commandID, payloadLength, payload, checksum};

        this->smartSerial->responseMapMutex.lock();
        this->smartSerial->responseMap[UUID] = response;
        this->smartSerial->responseMapMutex.unlock();

        this->reset();

        break;
    }
}
