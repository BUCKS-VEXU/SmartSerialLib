/* Noah Klein */

#include "main.h"

#include "SmartSerialLib/SmartSerial.hpp"
#include "SmartSerialLib/protocol/ProtocolDefinitions.hpp"
#include "liblvgl/llemu.hpp"

SmartSerial esp32Serial(7, 576000);

void onCenterButton() {
    int64_t pingResponse = esp32Serial.ping(0x7E, 1000);

    if (pingResponse < 0) {
        pros::lcd::print(0, "Ping error: %d", pingResponse);
    } else {
        pros::lcd::print(0, "Ping took %d microseconds", pingResponse);
    }
}

void onLeftButton() {
    GetPoseRequest poseRequest;
    int poseResponse =
        esp32Serial.sendAndDeserializeResponse(poseRequest, 1000);

    if (poseResponse != 0) {
        pros::lcd::print(1, "Get pose error: %d", poseResponse);
        return;
    } else {
        pose2d_t pose = poseRequest.getPose();
        pros::lcd::print(
            1, "Pose: (%.2f, %.2f, %.2f)", pose.x, pose.y, pose.theta);
    }
}

void onRightButton() {
    SmartSerialDiagnostic diagnostics = esp32Serial.getDiagnostics();
    pros::lcd::print(0, "Last UUID: %d", diagnostics.lastUUID);
    pros::lcd::print(1, "Current State: %d", diagnostics.currentState);
    pros::lcd::print(2, "Total bytes read: %d", diagnostics.totalBytesRead);
    pros::lcd::print(
        3, "Total bytes written: %d", diagnostics.totalBytesWritten);
    pros::lcd::print(4, "Read errors: %d", diagnostics.readErrors);
    pros::lcd::print(5, "Write errors: %d", diagnostics.writeErrors);
    pros::lcd::print(
        6, "Deserialization failures: %d", diagnostics.deserializationFailures);
}

/**
 * (Ideally) called once on program start
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::register_btn0_cb(onLeftButton);
    pros::lcd::register_btn1_cb(onCenterButton);
    pros::lcd::register_btn2_cb(onRightButton);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        pros::delay(20);
    }
}
