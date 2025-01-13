/* Noah Klein */

#include "main.h"

#include "SmartSerialLib/SmartSerial.hpp"
#include "SmartSerialLib/protocol/Requests/Requests.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
SmartSerial esp32Serial(0);

/**
 * (Ideally) called once on program start
 */
void initialize() {
    GetPoseRequest pose;
    esp32Serial.sendAndDeserializeResponse(pose, 1000);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "Ping took " << esp32Serial.ping(0xFE, 1000)
                      << " microseconds" << std::endl;
        }

        pros::delay(20); // Run for 20 ms then update
    }
}
