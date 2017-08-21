#include "STSL/OSXUtils.h"
#include <iostream>
#include <unistd.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>

using namespace std;

std::string OSXUtils::FindRobot() {
    return FindConnectedArduinos().front();
}

std::vector<std::string> OSXUtils::FindConnectedArduinos() {

    // TODO https://developer.apple.com/library/content/documentation/DeviceDrivers/Conceptual/WorkingWSerial/WWSerial_SerialDevs/SerialDevices.html

    return {};
}

void OSXUtils::Sleep(std::chrono::microseconds duration) {
    usleep(static_cast<useconds_t>(duration.count()));
}
