#include <iostream>
#include "STSL/RJRobot.h"

#ifdef __WIN32
#include "STSL/WindowsUtils.h"
#elif __linux__
#include "STSL/LinuxUtils.h"
#endif

using namespace std;

RJRobot::RJRobot()
    : serial_port_(io_service_)
{
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

    serial_port_.open(port_path);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(9600));
}

RJRobot::~RJRobot() {

}
