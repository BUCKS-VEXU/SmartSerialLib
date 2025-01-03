/* Noah Klein */

#ifndef SERIAL_ESP32_HPP
#define SERIAL_ESP32_HPP

#include "pros/rtos.hpp"
#include "pros/serial.hpp"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <unordered_map>
#include <vector>

#include "protocol/ProtocolDefinitions.hpp"
#include "protocol/Requests/Request.hpp"
#include "protocol/Requests/Requests.hpp" // IWYU pragma: keep

#include "ResponseStateMachine.hpp"

struct SmartSerialDiagnostic {
    size_t responseMapSize;
    uint8_t currentUUID;
    State currentState;
    size_t totalBytesRead;
    size_t totalBytesWritten;
    size_t readErrors;
    size_t writeErrors;
    size_t deserializationFailures;
};

class SmartSerial {
  private:
    pros::Serial serial;
    pros::Task serialReader;

    uint8_t currentUUID = 0;

    // Diagnostic information
    size_t totalBytesRead = 0;
    size_t totalBytesWritten = 0;
    size_t readErrors = 0;
    size_t writeErrors = 0;
    size_t deserializationFailures = 0;

    void serialReader_fn(void *ignore);

    // TODO this probably doesn't need to be a map, a small buffer might suffice
    pros::Mutex responseMutex;
    std::unordered_map<uint8_t, SerialResponse> responseMap;
    std::unordered_map<uint8_t, pros::Task> waitingTasks;

    friend class ResponseStateMachine;
    ResponseStateMachine stateMachine;

    int addResponse(SerialResponse *response);

  public:
    SmartSerial(const int port, const int baudrate = 115200) :
        serial(port, baudrate),
        responseMutex(),
        responseMap(),
        waitingTasks(),
        stateMachine(this),
        // TODO God only knows if this works
        serialReader(
            [](void *param) -> void {
                static_cast<SmartSerial *>(param)->serialReader_fn(param);
            },
            this) {};
    ~SmartSerial() = default;

    /**
     * @brief Sends a request, updating the UUID of `request` and incrementing
     * `this->currentUUID`
     *
     * @param request the request to send
     * @return the request's UUID if the request was successfully sent
     * @return -1 if the request was not successfully sent
     */
    int sendRequest(Request &request);

    // TODO update this documentation for notifications

    /**
     * @brief Returns a pointer to the `SerialResponse` with the given UUID,
     * or nullptr if said response does not exist.
     *
     * @param UUID
     * @return SerialResponse* if the response is in the map
     * @return nullptr if the response is not in the map
     */
    SerialResponse *getResponse(uint8_t UUID);

    /**
     * @brief Blocking operation that waits for a response with the given UUID,
     * then returns a pointer to the response in the response map
     *
     * @param UUID the UUID of the response to wait for
     * @param busyWaitMs the time to busy wait for a response before switching
     * to a pros::delay loop
     * @param timeoutMs the time to wait for a response before returning nullptr
     * @return SerialResponse*, pointer to the response in the response map, or
     * nullptr if the response was not received in time
     */
    SerialResponse *waitForResponse(uint8_t UUID, uint32_t timeoutMs = 1000);

    /**
     * @brief Attempts to remove a response from the response map with the given
     * UUID
     *
     * @param UUID the UUID of the response to remove
     * @return true if a response was removed from the map
     * @return false if no response was removed from the map
     */
    bool removeResponseFromMap(uint8_t UUID);

    /**
     * @brief A blocking operation that sends a request, then deserializes the
     * response and updates response fields of the passed Request instance. The
     * request sent will be removed from the response map after deserialization.
     *
     * @param request
     * @return true if the response was successfully sent, received and
     * deserialized
     * @return false if any part of the sending or deserialization process
     * failed
     */
    bool sendAndDeserializeResponse(Request &request,
                                    uint32_t busyWaitMs = 5,
                                    uint32_t timeoutMs = 1000);

    /**
     * @brief Sends a ping request to the ESP32
     *
     * @param pingByte the byte to send in the ping request
     * @param timeoutMs the time to wait for a response
     * @return u_int64_t the number of microseconds it took to receive a
     * response
     * @return 0 if no response was received in time
     */
    u_int64_t ping(uint8_t pingByte, uint32_t timeoutMs);

    /**
     * @brief Returns diagnostic information about this SmartSerial instance
     *
     * @return SmartSerialDiagnostic
     */
    SmartSerialDiagnostic getDiagnostics();
};

#endif // SERIAL_ESP32_HPP
