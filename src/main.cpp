/* Noah Klein */

#include "main.h"

#include "SmartSerialLib/SmartSerial.hpp"

SmartSerial esp32Serial(0);

/**
 * (Ideally) called once on program start
 */
void initialize() {
    std::cout << "Ping took " << esp32Serial.ping(0xFE, 1000) << " microseconds"
              << std::endl;

    GetPoseRequest pose;
    esp32Serial.sendAndDeserializeResponse(pose, 1000);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        pros::delay(20); // Run for 20 ms then update
    }
}
