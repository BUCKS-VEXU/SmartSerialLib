/* Noah Klein */

#include "SmartSerial.hpp"
#include "ResponseStateMachine.hpp"

void ResponseStateMachine::onChecksumFail() const {
    std::cout << "Checksum mismatch\n Expected:" << calculatedChecksum
              << "\n Received: " << checksum << "\n";
}

void ResponseStateMachine::onMissingStartMarker(uint8_t byte) const {
    std::cout << "Expected start marker, " << std::hex << START_MARKER
              << ", but received " << std::hex << byte << "\n";
}

void ResponseStateMachine::onMissingEndMarker(uint8_t byte) const {
    std::cout << "Expected end marker, " << std::hex << END_MARKER
              << ", but received " << std::hex << byte << "\n";
}

void ResponseStateMachine::onMessageComplete() const {
    // TODO handle error responses (commandID = ERROR)

    auto payload = std::make_shared<std::vector<uint8_t>>(
        payloadBuffer, payloadBuffer + payloadLength);
    SerialResponse response = {
        UUID, commandID, payloadLength, payload, checksum};

    this->smartSerial->addResponse(response);
}
