#include "STSL/OSXUtils.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>

using namespace std;

std::string OSXUtils::FindRobot() {
    return FindConnectedArduinos().front();
}

std::vector<std::string> OSXUtils::FindConnectedArduinos() {
    auto ls_stream = popen("ls /dev/tty.*", "r");

    if(ls_stream == nullptr) {
        return {};
    }

    char buffer[1024];
    string response;
    while(fgets(buffer, sizeof(buffer), ls_stream) != nullptr) {
        response += string{buffer};
    }

    pclose(ls_stream);

    vector<string> serialports;

    string delim{"\n"};
    auto start = 0ul;
    auto end = response.find(delim);
    while(end != std::string::npos) {
        serialports.emplace_back(response.substr(start, end-start));
        start = end + delim.size();
        end = response.find(delim, start);
    }

    auto is_arduino_predicate = [](const string &port) {
        return port.empty() || port.substr(9,8) != "usbmodem";
    };

    if(!serialports.empty()) {
        serialports.erase(std::remove_if(serialports.begin(), serialports.end(), is_arduino_predicate));
    }

    return serialports;
}

void OSXUtils::Sleep(std::chrono::microseconds duration) {
    usleep(static_cast<useconds_t>(duration.count()));
}
