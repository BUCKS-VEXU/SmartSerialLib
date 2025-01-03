/* Noah Klein */

#include "pros/adi.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <vector>

#include "SmartSerialLib/SmartSerial.hpp"

void SmartSerial::serialReader_fn(void *ignore) {
    uint32_t startTime = pros::millis();

    while (true) {
        // TODO maybe this needs a timeout so that it can't read indefinitely
        // Read all available bytes from the serial port
        while (serial.get_read_avail() > 0) {
            uint32_t byte = serial.read_byte();
            if (byte == PROS_ERR || byte == -1) {
                std::cout << "Error reading byte from serial port\n";
                readErrors++;
                continue;
            }
            stateMachine.loop(static_cast<uint8_t>(byte));
        }
        pros::Task::delay_until(&startTime, 10);
    }
}

int SmartSerial::addResponse(SerialResponse &response) {
    const uint8_t &UUID = response.UUID;

    std::lock_guard<pros::Mutex> lock(responseMutex);

    bool hadPayload = payloads[UUID].has_value();
    // TODO i'm not certain that this properly constructs a new shared pointer,
    // it probably does.
    payloads[UUID] = response.payload;

    // Is there a task waiting for a response with this UUID, notify it
    if (waitingTasks[UUID].has_value()) {
        pros::Task waitingTask = waitingTasks[UUID].value();
        waitingTasks[UUID] = std::nullopt;
    }

    if (hadPayload) {
        std::cout << "Response with UUID " << UUID
                  << " already exists in map\n";
        return -1;
    }

    return 0;
}

bool SmartSerial::removePayload(uint8_t UUID) {
    std::lock_guard<pros::Mutex> lock(responseMutex);
    bool hadPayload = payloads[UUID].has_value();
    payloads[UUID] = std::nullopt;
    return hadPayload;
}

int SmartSerial::sendRequest(Request &request) {
    request.setUUID(currentUUID++);
    std::vector<uint8_t> serializedRequest = request.serializeRequest();

    // TODO maybe I need to ensure synchronized serial writes to avoid EACCES

    const size_t requestLength = serializedRequest.size();
    int bytesWritten = serial.write(serializedRequest.data(), requestLength);

    if (bytesWritten != requestLength) {
        writeErrors++;
        return -1;
    }

    totalBytesWritten += bytesWritten;
    return request.getUUID();
}

payload_t SmartSerial::getPayload(uint8_t UUID) {
    std::lock_guard<pros::Mutex> lock(responseMutex);

    payload_t payload = payloads[UUID].value_or(nullptr);
    payloads[UUID] = std::nullopt;
    return payload;
}

payload_t SmartSerial::waitForResponse(uint8_t UUID, uint32_t timeoutMs) {
    using namespace pros;

    this->waitingTasks[UUID] = Task::current();

    Task::current().notify_clear();

    int beforeClear = Task::current().notify_take(true, timeoutMs);

    return beforeClear ? this->getPayload(UUID) : nullptr;
}

bool SmartSerial::sendAndDeserializeResponse(Request &request,
                                             uint32_t timeoutMs) {
    // Step 1: Send request
    int sentUUID = this->sendRequest(request);
    if (sentUUID == -1) {
        return false; // Sending failed
    }

    // Step 2: Wait for a response
    payload_t payload = this->waitForResponse(sentUUID, timeoutMs);
    if (payload == nullptr) {
        return false; // Response not received in time
    }

    // Step 3: Deserialize the response
    try {
        request.deserializeResponsePayload(*payload);
    } catch (const std::exception &e) {
        deserializationFailures++;
        return false; // Deserialization failed
    }

    return true;
}

uint64_t SmartSerial::ping(uint8_t pingByte, uint32_t timeoutMs) {
    PingRequest ping(pingByte);
    ping.setUUID(currentUUID++);
    const uint64_t startTime = pros::micros();
    if (!sendAndDeserializeResponse(ping, timeoutMs)) {
        return 0;
    }
    const uint64_t pingTime = pros::micros() - startTime;

    // Return 0 if the ping response was not the expected value
    return (pingByte == ping.getPingResponse()) ? pingTime : 0;
}

SmartSerialDiagnostic SmartSerial::getDiagnostics() {
    return {.currentUUID = currentUUID,
            .currentState = stateMachine.getCurrentState(),
            .totalBytesRead = totalBytesRead,
            .totalBytesWritten = totalBytesWritten,
            .readErrors = readErrors,
            .writeErrors = writeErrors,
            .deserializationFailures = deserializationFailures};
}
