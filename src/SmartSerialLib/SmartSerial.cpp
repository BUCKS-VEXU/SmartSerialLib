/* Noah Klein */

#include "pros/adi.h"
#include <cstdint>
#include <iostream>
#include <mutex>

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

int SmartSerial::sendRequest(Request &request) {
    request.setUUID(currentUUID++);
    std::vector<uint8_t> serializedRequest = request.serializeRequest();

    size_t requestLength = serializedRequest.size();
    size_t bytesWritten = serial.write(serializedRequest.data(), requestLength);

    if (bytesWritten != requestLength) {
        writeErrors++;
        return -1;
    }

    totalBytesWritten += bytesWritten;
    return request.getUUID();
}

SerialResponse *SmartSerial::getResponse(uint8_t UUID) {
    // TODO, make sure this works
    std::lock_guard<pros::Mutex> lock(responseMapMutex);
    auto it = responseMap.find(UUID);
    return (it != responseMap.end()) ? &it->second : nullptr;
}

// TODO verify this works
SerialResponse *SmartSerial::waitForResponse(uint8_t UUID,
                                             uint32_t busyWaitMs,
                                             uint32_t timeoutMs) {
    const uint32_t startTime = pros::millis();
    const uint32_t busyWaitEnd = startTime + busyWaitMs;

    // "Busy wait" for a response for busyWaitMs milliseconds
    while (pros::millis() < busyWaitEnd) {
        if (SerialResponse *response = getResponse(UUID)) {
            return response;
        }
        pros::Task::delay(1);
    }

    // Wait for a response for timeoutMs milliseconds since startTime
    while (pros::millis() < startTime + timeoutMs) {
        if (SerialResponse *response = getResponse(UUID)) {
            return response;
        }
        pros::Task::delay(10);
    }

    return nullptr;
}

bool SmartSerial::removeResponseFromMap(uint8_t UUID) {
    // TODO, ensure this works
    std::lock_guard<pros::Mutex> lock(responseMapMutex);

    auto it = responseMap.find(UUID);
    if (it != responseMap.end()) {
        responseMap.erase(it);
        return true;
    }
    return false;
}

bool SmartSerial::sendAndDeserializeResponse(Request &request,
                                             uint32_t busyWaitMs,
                                             uint32_t timeoutMs) {
    // Step 1: Send request
    int sentUUID = this->sendRequest(request);
    if (sentUUID == -1) {
        return false; // Sending failed
    }

    // Step 2: Wait for a response
    SerialResponse *response =
        this->waitForResponse(sentUUID, busyWaitMs, timeoutMs);
    if (response == nullptr) {
        return false; // Response not received in time
    }

    // Step 3: Deserialize the response
    try {
        request.deserializeResponsePayload(*response->payload);
    } catch (const std::exception &e) {
        deserializationFailures++;
        return false; // Deserialization failed
    }

    // Step 4: Remove the response from map, return false if it wasn't removed
    return this->removeResponseFromMap(request.getUUID());
}

uint64_t SmartSerial::ping(uint8_t pingByte, uint32_t timeoutMs) {
    PingRequest ping(pingByte);
    ping.setUUID(currentUUID++);
    const uint64_t startTime = pros::micros();
    return sendAndDeserializeResponse(ping, timeoutMs)
               ? (pros::micros() - startTime)
               : 0;
}

SmartSerialDiagnostic SmartSerial::getDiagnostics() {
    return {.responseMapSize = responseMap.size(),
            .currentUUID = currentUUID,
            .currentState = stateMachine.getCurrentState(),
            .totalBytesRead = totalBytesRead,
            .totalBytesWritten = totalBytesWritten,
            .readErrors = readErrors,
            .writeErrors = writeErrors,
            .deserializationFailures = deserializationFailures};
}
