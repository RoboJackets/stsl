#include <iostream>
#include <cassert>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <windows.h>
    #include <ws2tcpip.h>
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <unistd.h>
#endif

#include <STSL/RJRobot.h>
#include <STSL/Sleep.h>

using namespace std;

constexpr const char* HUZZAH_IP = "10.10.10.1";

RJRobot::RJRobot() {
    cout << "Initializing..." << endl;

#ifdef _WIN32
    WSADATA wsa_data;
    if(auto ret = WSAStartup(MAKEWORD(1,1), &wsa_data); ret != 0) {
        handleError(ret, "WSAStartup failed");
    }
#endif

    socket_handle = socket(AF_INET, SOCK_STREAM, 0);

    if(!isValidSocketHandle(socket_handle)) {
        handleError("Unable to create socket");
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(80);

    if(auto ret = inet_pton(AF_INET, HUZZAH_IP, &serv_addr.sin_addr); ret != 1) {
        handleError("inet_pton failed");
    }

    if(auto ret = connect(socket_handle, reinterpret_cast<sockaddr*>(&serv_addr), sizeof(serv_addr)); ret != 0) {
        handleError("connect failed");
    }

    cout << "Robot ready!" << endl;
}

RJRobot::~RJRobot() {
#ifdef _WIN32
    if(auto ret = shutdown(socket_handle, SD_BOTH); ret != 0) {
        handleError("shutdown failed");
    }
    if(auto ret = closesocket(socket_handle); ret != 0) {
        handleError("closesocket failed");
    }
    if(auto ret = WSACleanup(); ret != 0) {
        handleError(ret, "WSACleanup failed");
    }
#else
    if(auto ret = shutdown(socket_handle, SHUT_RDWR); ret != 0) {
        handleError("shutdown failed");
    }
    if(auto ret = close(socket_handle); ret != 0) {
        handleError("close failed");
    }
#endif
}

void RJRobot::SetMotor(const Motor &port, const int &speed) {
    assert(speed <= 255);
    assert(speed >= -255);
    string motorName;
    switch(port) {
    case Motor::LEFT:
        motorName = "Left";
        break;
    case Motor::RIGHT:
        motorName = "Right";
        break;
    case Motor::LIFT:
        motorName = "Lift";
        break;
    }
    sendCommand(("SetMotor" + motorName + to_string(speed) + "\n").c_str());
}

void RJRobot::StopMotors() {
    sendCommand("StopMotors");
}

int RJRobot::GetLightValue(const LightSensor &sensor) {
    sendCommand((sensor == LightSensor::CENTER ? "GetLightCenter" : "GetLightRight"));

    auto response = getResponse();

    return std::stoi(response);
}

double RJRobot::GetUltrasonicDistance() {
    sendCommand("GetUltrasonic");

    auto response = getResponse();

    return std::stod(response);
}

double RJRobot::GetProximity() {
    sendCommand("GetProximity");
    auto response = getResponse();
    return std::stod(response);
}

Gesture RJRobot::GetGesture() {
    sendCommand("GetGesture");
    auto response = getResponse();
    if(response == "UP") return Gesture::UP;
    if(response == "DOWN") return Gesture::DOWN;
    if(response == "LEFT") return Gesture::LEFT;
    if(response == "RIGHT") return Gesture::RIGHT;
    return Gesture::NONE;
}

Color RJRobot::GetColor() {
    sendCommand("GetColor");
    auto response = getResponse();
    auto firstSpace = response.find(' ');
    auto secondSpace = response.find(' ', firstSpace+1);
    auto thirdSpace = response.find(' ', secondSpace+1);
    auto r = std::stoi(response.substr(0, firstSpace));
    auto g = std::stoi(response.substr(firstSpace+1,secondSpace-(firstSpace+1)));
    auto b = std::stoi(response.substr(secondSpace+1,thirdSpace-(secondSpace+1)));
    auto c = std::stoi(response.substr(thirdSpace+1));
    if(r > b) return Color::RED;
    if(b > r) return Color::BLUE;
    return Color::UNKNOWN;
}

void RJRobot::Wait(std::chrono::microseconds duration) {
    sleep(duration);
}

bool RJRobot::isValidSocketHandle(const socket_t &socket) {
#ifdef _WIN32
    return socket != INVALID_SOCKET;
#else
    return socket >= 0;
#endif
}

void RJRobot::sendCommand(const char *command) {
    auto remaining_bytes = std::strlen(command);
    auto current_pos = 0;
    while(remaining_bytes > 0) {
        auto bytes_sent = send(socket_handle, command + current_pos, remaining_bytes, 0);
        if(bytes_sent >= 0) {
            remaining_bytes -= bytes_sent;
            current_pos += bytes_sent;
        } else {
            handleError("send failed");
        }
    }
}

std::string RJRobot::getResponse() {
    std::string response;
    constexpr auto buffer_size = 10ul;
    auto buffer = new char[buffer_size];
    auto bytes_received = recv(socket_handle, buffer, buffer_size, 0);
    if(bytes_received == -1) {
        handleError("recv failed");
    } else if(bytes_received == 0) {
        std::cout << "Unexpected disconnect! (recv() returned 0)\n";
        return response;
    } else {
        response.insert(response.size(), buffer, bytes_received);
        for(auto i = 0; i < bytes_received; i++) {
            if(buffer[i] == '\n') {
                return response.erase(i);
            }
        }
    }
}

void RJRobot::handleError(const char *message) {
#ifdef _WIN32
    auto error_code = WSAGetLastError();
#else
    auto error_code = errno;
#endif
    throw std::system_error(error_code, std::generic_category(), message);
}

void RJRobot::handleError(int retval, const char *message) {
    throw std::system_error(retval, std::generic_category(), message);
}
