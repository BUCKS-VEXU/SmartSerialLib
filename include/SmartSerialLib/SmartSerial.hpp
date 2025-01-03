/* Noah Klein */

#ifndef SERIAL_ESP32_HPP
#define SERIAL_ESP32_HPP

#include "pros/rtos.hpp"
#include "pros/serial.hpp"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

#include "protocol/ProtocolDefinitions.hpp"
#include "protocol/Requests/Request.hpp"
#include "protocol/Requests/Requests.hpp" // IWYU pragma: keep

#include "ResponseStateMachine.hpp"

struct SmartSerialDiagnostic {
    uint8_t currentUUID;
    State currentState;
    int availableBytes;
    size_t totalBytesRead;
    size_t totalBytesWritten;
    size_t readErrors;
    size_t writeErrors;
    size_t deserializationFailures;
};

class SmartSerial {
  private:
    pros::Mutex serialMutex;
    pros::Serial serial;

    pros::Task serialReader;
    void serialReader_fn(void *ignore);

    // TODO Wisely incrementing currentUUID could allow for
    // TODO fewer payload overwrites
    uint8_t currentUUID = 0;
    static constexpr size_t MAX_RESPONSES = UINT8_MAX + 1;

    // Diagnostic information
    int availableBytes = 0;
    size_t totalBytesRead = 0;
    size_t totalBytesWritten = 0;
    size_t readErrors = 0;
    size_t writeErrors = 0;
    size_t deserializationFailures = 0;

    // TODO weigh the synchronization against the time cost of a mutex here
    pros::Mutex payloadMutex;
    // ! these two maps currently take up 5120 bytes of memory
    // This could be made a lot smaller if I don't care about asynchronous
    // response gets
    // TODO tighter bonding between these two could be good
    std::array<std::optional<payload_t>, MAX_RESPONSES> payloads;
    std::array<std::optional<pros::Task>, MAX_RESPONSES> waitingTasks;

    friend class ResponseStateMachine;
    ResponseStateMachine stateMachine;

    /**
     * @brief Adds the response's payload to the payloads array. Notifies a
     * task, if there is one waiting on this UUID.
     *
     * @param response
     * @return 0 if the payload was successfully added
     * @return -1 if the payload overwrote a different payload in the array
     */
    int addResponse(SerialResponse &response);

    /**
     * @brief Update currentUUID to the next available.
     * Settle for (currentUUID + 1) if none are found.
     *
     * @return uint8_t the value currentUUID was updated to
     */
    uint8_t nextUUID();

  public:
    SmartSerial(const int port, const int baudrate = 115200) :
        serial(port, baudrate),
        serialMutex(),
        payloadMutex(),
        payloads(),
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
     * @return -1 if PROS_ERR was returned by serial.write()
     * @return -2 if the whole serialized message was not written
     */
    int sendRequest(Request &request);

    /**
     * @brief Returns a shared pointer to the payload at the given UUID,
     * or nullptr if said payload does not exist. Removes the shared pointer in
     * the array before returning.
     *
     * @param UUID
     * @return payload_t if the payload is present
     * @return nullptr if the payload is not present
     */
    payload_t getPayload(uint8_t UUID);

    /**
     * @brief Blocking operation that waits for a response with the given UUID,
     * then returns a shared pointer to the response's payload. Removes the
     * shared pointer in the array before returning.
     *
     * @param UUID the UUID of the response to wait for
     * @param timeoutMs the time to wait for a response before returning nullptr
     * @return payload, shared pointer to the response's payload
     * @return nullptr if the response was not received in time
     */
    payload_t waitForResponse(uint8_t UUID, uint32_t timeoutMs = 1000);

    /**
     * @brief Sets the payload at UUID to std::nullopt
     *
     * @param UUID the UUID of the payload to make std::nullopt
     * @return true if there was a payload to remove
     * @return false if the payload was already std::nullopt
     */
    bool removePayload(uint8_t UUID);

    /**
     * @brief A blocking operation that sends a request, then deserializes the
     * response and updates response fields of the passed Request instance. The
     * request sent will be removed from the response map after deserialization.
     *
     * @param request
     * @return 0 if the process was successful
     * @return -1 if sending the request failed
     * @return -2 if the response was not received in time
     * @return -3 if deserialization failed
     */
    int sendAndDeserializeResponse(Request &request, uint32_t timeoutMs = 1000);

    /**
     * @brief Sends a ping request with the given byte, then verifies that that
     * same byte was returned
     *
     * @param pingByte the byte to send in the ping request
     * @param timeoutMs the time to wait for a response
     * @return int64_t the number of microseconds it took to receive a
     * response
     * @return -1 if sending the ping failed
     * @return -2 if the response was not received in time
     * @return -3 if deserializing the ping failed
     * @return -4 if the byte received does not match `pingByte`
     */
    int64_t ping(uint8_t pingByte, uint32_t timeoutMs);

    /**
     * @brief Returns diagnostic information about this SmartSerial instance
     *
     * @return SmartSerialDiagnostic
     */
    SmartSerialDiagnostic getDiagnostics();
};

#endif // SERIAL_ESP32_HPP
