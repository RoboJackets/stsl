#include <iostream>
#include <cassert>
#include "STSL/RJRobot.h"
#include "STSL/Sleep.h"

using namespace std;

RJRobot::RJRobot() {
    cout << "Initializing..." << endl;

    cout << "Robot ready!" << endl;
}

RJRobot::~RJRobot() {
//    serial_port_.Close();
}

bool RJRobot::IsButtonPressed() {
//    serial_port_.Write("GetButton\n");
//    auto response = serial_port_.ReadLine();
//    return (response[0] == '1');
    return false;
}

uint8_t RJRobot::LightValue() {
//    serial_port_.Write("GetLight\n");
//    auto response = serial_port_.ReadLine();
//    return static_cast<uint8_t>(std::stoul(response));
    return 0;
}

void RJRobot::SetFloodlight(bool on) {
//    serial_port_.Write(string{"SetFloodlight"} + (on ? "On" : "Off") + "\n");
}

void RJRobot::SetMotor(const MotorPort &port, const int &speed) {
    assert(speed <= 255);
    assert(speed >= -255);
//    serial_port_.Write(string{"SetMotor"} + (port == MotorPort::A ? "A" : "B") + to_string(speed) + "\n");
}

void RJRobot::StopMotors() {
//    serial_port_.Write("StopMotors");
}

void RJRobot::Wait(std::chrono::microseconds duration) {
    sleep(duration);
}
