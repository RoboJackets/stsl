#include "STSL/OSXUtils.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>

using namespace std;

std::string OSXUtils::FindRobot() {
    auto arduinos = FindConnectedArduinos();
    if(arduinos.empty())
        return "";
    return arduinos.front();
}

std::vector<std::string> OSXUtils::FindConnectedArduinos() {
    auto ls_stream = popen("ls /dev/cu.usbmodem*", "r");

    if(ls_stream == nullptr) {
        return {};
    }

    char buffer[1024];
    string response;
    while(fgets(buffer, sizeof(buffer), ls_stream) != nullptr) {
        response += string{buffer};
    }

    pclose(ls_stream);

    if(response == "ls: /dev/cu.usbmodem*: No such file or directory") {
        // No ports found;
        return {};
    }

    vector<string> serialports;

    string delim{"\n"};
    auto start = 0ul;
    auto end = response.find(delim);
    while(end != std::string::npos) {
        serialports.emplace_back(response.substr(start, end-start));
        start = end + delim.size();
        end = response.find(delim, start);
    }

    auto is_empty_predicate = [](const string &port) {
        return port.empty();
    };

    if(!serialports.empty()) {
        auto newEnd = std::remove_if(serialports.begin(), serialports.end(), is_empty_predicate);
        if(newEnd != serialports.end())
            serialports.erase(newEnd);
    }

    return serialports;
}
