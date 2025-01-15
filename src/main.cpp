/* Noah Klein */

#include "main.h"

#include "SmartSerialLib/SmartSerial.hpp"

// SmartSerial esp32Serial(15);
pros::Serial serial(8, 9600);

static uint8_t byte = 0x00;

void onCenterButton() {
    int writeVal = serial.write_byte(++byte);
    if (writeVal == PROS_ERR)
        std::cout << "Write error" << std::endl;
    else
        std::cout << "Wrote " << static_cast<int>(byte) << std::endl;
}

/**
 * (Ideally) called once on program start
 */
void initialize() {
    // int pingResponse = esp32Serial.ping(0xAE, 1000);
    // std::cout << "Ping took " << pingResponse << " microseconds" <<
    // std::endl;'
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(onCenterButton);
    std::cout << "Serial is installed " << serial.is_installed() << std::endl;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

    while (true) {
        // int pingResponse = esp32Serial.ping(0xAE, 1000);
        // std::cout << "Ping took " << pingResponse << " microseconds"
        //           << std::endl;

        if (serial.get_read_avail()) {
            uint8_t readByte = serial.read_byte();
            std::cout << "Read: " << readByte << std::endl;
            pros::lcd::print(0, "Read: %d", readByte);
        }

        pros::delay(20);
    }
}
