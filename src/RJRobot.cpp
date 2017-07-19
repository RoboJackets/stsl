#include <iostream>
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

    serial_port_.Open(port_path, 9600);

    os_utils_->Sleep(2s);

    cout << "Robot ready!" << endl;
}

RJRobot::~RJRobot() {
    serial_port_.Close();
}

bool RJRobot::IsButtonPressed() {
    serial_port_.Write("GetButton");
    auto response = serial_port_.ReadLine();
    return (response[0] == '1');
}

void RJRobot::Wait(std::chrono::microseconds duration) {
    os_utils_->Sleep(duration);
}
