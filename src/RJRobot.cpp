#include <iostream>
#include <cassert>
#include "STSL/RJRobot.h"

#ifdef __WIN32
#include "STSL/WindowsUtils.h"
#elif __linux__
#include "STSL/LinuxUtils.h"
#endif

using namespace std;

RJRobot::RJRobot() {
#ifdef __WIN32
    os_utils_.reset(new WindowsUtils);
#elif __linux__
    os_utils_.reset(new LinuxUtils);
#endif

    auto port_path = os_utils_->FindRobot();

    if(port_path.empty()) {
        cerr << "Could not find robot. Please make sure it is plugged in via USB." << endl;
        return;
    }

    cout << "Found robot at " << port_path << endl;
    cout << "Initializing..." << endl;

    if(!serial_port_.Open(port_path, 9600)) {
        cerr << "Failed to connect to robot!" << endl;
        return;
    }

    os_utils_->Sleep(2s);

    cout << "Robot ready!" << endl;
}

RJRobot::~RJRobot() {
    serial_port_.Close();
}

bool RJRobot::IsButtonPressed() {
    serial_port_.Write("GetButton\n");
    auto response = serial_port_.ReadLine();
    return (response[0] == '1');
}

uint8_t RJRobot::LightValue() {
    serial_port_.Write("GetLight\n");
    auto response = serial_port_.ReadLine();
    return static_cast<uint8_t>(std::stoul(response));
}

void RJRobot::SetFloodlight(bool on) {
    serial_port_.Write(string{"SetFloodlight"} + (on ? "On" : "Off") + "\n");
}

void RJRobot::SetMotor(const MotorPort &port, const int &speed) {
    assert(speed <= 255);
    assert(speed >= -255);
    serial_port_.Write(string{"SetMotor"} + (port == MotorPort::A ? "A" : "B") + to_string(speed) + "\n");
}

void RJRobot::StopMotors() {
    SetMotor(MotorPort::A, 0);
    SetMotor(MotorPort::B, 0);
}

void RJRobot::Wait(std::chrono::microseconds duration) {
    os_utils_->Sleep(duration);
}
