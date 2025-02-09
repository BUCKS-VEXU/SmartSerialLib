/* Noah Klein */

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <vector>

#include "pros/adi.h"
#include "pros/rtos.hpp"

#include "SmartSerialLib/SmartSerial.hpp"

void SmartSerial::serialReader_fn(void *ignore) {
    // TODO something with this
    uint32_t lastByteTime = pros::millis();

    while (true) {
        // TODO clean up mutex usage
        // Read all available bytes from the serial port
        serialMutex.lock();
        this->availableBytes = serial.get_read_avail();
        serialMutex.unlock();
        for (; availableBytes > 0; availableBytes--) {
            serialMutex.lock();
            uint32_t byte = serial.read_byte();
            serialMutex.unlock();

            totalBytesRead++;

#if SERIAL_DEBUG
            std::cout << "Read byte: " << std::hex << static_cast<int>(byte)
                      << "\n";
#endif

            if (byte == PROS_ERR || byte == -1) {
                std::cout << "Error reading byte from serial port\n";
                readErrors++;
            }

            stateMachine.loop(static_cast<uint8_t>(byte));
        }

        pros::Task::delay(10);
    }
}

int SmartSerial::addResponse(SerialResponse &response) {
    const uint8_t &UUID = response.UUID;

    std::lock_guard<pros::Mutex> lock(payloadMutex);

    bool hadPayload = payloads[UUID].has_value();
    payloads[UUID] = response.payload;

    // Is there a task waiting for a response with this UUID? Notify it
    if (waitingTasks[UUID].has_value()) {
        // ! It is the waiting task's responsibility to nullopt its array index
        waitingTasks[UUID].value().notify();
    }

    if (hadPayload) {
#if SERIAL_DEBUG
        std::cout << "Payload already present at index " << UUID << "\n";
#endif
        return -1;
    }

    return 0;
}

uint8_t SmartSerial::nextUUID() {
    const uint8_t first = lastUUID + 1;
    uint8_t next = first;

    // Search for the first available index, settle if none are found
    if (payloads[first].has_value()) {
        do {
            next++;
        } while (payloads[next].has_value() && next != first);
    }

    lastUUID = next;
    return next;
}

bool SmartSerial::removePayload(uint8_t UUID) {
    std::lock_guard<pros::Mutex> lock(payloadMutex);
    bool hadPayload = payloads[UUID].has_value();
    payloads[UUID] = std::nullopt;
    waitingTasks[UUID] = std::nullopt;
    return hadPayload;
}

int SmartSerial::sendRequest(Request &request) {
    request.setUUID(nextUUID());
    std::vector<uint8_t> serializedRequest = request.serializeRequest();

    const size_t requestLength = serializedRequest.size();

    serialMutex.lock();
    int bytesWritten = serial.write(serializedRequest.data(), requestLength);
    serialMutex.unlock();

    if (bytesWritten == PROS_ERR) {
        writeErrors++;
        return -1;
    }
    if (bytesWritten != requestLength) {
        writeErrors++;
        return -2;
    }

    totalBytesWritten += bytesWritten;
    return request.getUUID();
}

payload_t SmartSerial::getPayload(uint8_t UUID) {
    std::lock_guard<pros::Mutex> lock(payloadMutex);

    payload_t payload = payloads[UUID].value_or(nullptr);
    payloads[UUID] = std::nullopt;
    return payload;
}

payload_t SmartSerial::waitForResponse(uint8_t UUID, uint32_t timeoutMs) {
    using namespace pros;

    waitingTasks[UUID] = Task::current();

    Task::current().notify_clear();
    int notified = Task::current().notify_take(true, timeoutMs);

    // nullopt array index so this task is not notified again
    waitingTasks[UUID] = std::nullopt;

    return notified ? this->getPayload(UUID) : nullptr;
}

int SmartSerial::sendAndDeserializeResponse(Request &request,
                                            uint32_t timeoutMs) {
    // Step 1: Send request
    const int sentUUID = this->sendRequest(request);
    if (sentUUID == -1) {
        return -1; // Sending failed
    }

    // Step 2: Wait for a response
    payload_t payload =
        this->waitForResponse(static_cast<uint8_t>(sentUUID), timeoutMs);
    if (payload == nullptr) {
        return -2; // Response not received in time
    }

    // Step 3: Deserialize the response
    try {
        request.deserializeResponsePayload(*payload);
    } catch (const std::exception &e) {
        deserializationFailures++;
        return -3; // Deserialization failed
    }

    return 0;
}

int64_t SmartSerial::ping(uint8_t pingByte, uint32_t timeoutMs) {
    PingRequest ping(pingByte);

    const uint64_t startTime = pros::micros();
    int errorCode;
    if ((errorCode = sendAndDeserializeResponse(ping, timeoutMs))) {
        return errorCode;
    }
    const uint64_t pingTime = pros::micros() - startTime;

#if SERIAL_DEBUG
    std::cout << std::hex << static_cast<int>(ping.getPingResponse()) << "\n";
#endif

    // Return -4 if the ping response was not the expected value
    return (pingByte == ping.getPingResponse()) ? pingTime : -4;
}

SmartSerialDiagnostic SmartSerial::getDiagnostics() {
    return {.lastUUID = lastUUID,
            .currentState = stateMachine.getCurrentState(),
            .totalBytesRead = totalBytesRead,
            .totalBytesWritten = totalBytesWritten,
            .readErrors = readErrors,
            .writeErrors = writeErrors,
            .deserializationFailures = deserializationFailures};
}
