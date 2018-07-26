#include <iostream>
#include <cassert>
#include "STSL/RJRobot.h"
#include "STSL/Sleep.h"

#ifdef _WIN32
    #include <winsock2.h>
    #include <Ws2tcpip.h>
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <unistd.h>
#include <STSL/RJRobot.h>

#endif

using namespace std;

constexpr const char* HUZZAH_IP = "10.10.10.1";

RJRobot::RJRobot() {
    cout << "Initializing..." << endl;

#ifdef _WIN32
    WSADATA wsa_data;
    if(auto ret = WSAStartup(MAKEWORD(1,1), &wsa_data); ret != 0) {
        throw std::system_error(ret, std::generic_category(), "WSAStartup failed!");
    }
#endif

    socket_handle = socket(AF_INET, SOCK_STREAM, 0);

    if(!isValidSocketHandle(socket_handle)) {
#ifdef _WIN32
        auto error_code = WSAGetLastError();
#else
        auto error_code = errno;
#endif
        throw std::system_error(error_code, std::generic_category(), "Unable to create socket.");
    }

    sockaddr_in serv_addr{0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(80);

    if(auto ret = inet_pton(AF_INET, HUZZAH_IP, &serv_addr.sin_addr); ret != 0) {
        // TODO error handling
    }

    if(auto ret = connect(socket_handle, reinterpret_cast<sockaddr*>(&serv_addr), sizeof(serv_addr)); ret != 0) {
        // TODO error handling
    }

    cout << "Robot ready!" << endl;
}

RJRobot::~RJRobot() {
#ifdef _WIN32
    if(auto ret = shutdown(socket_handle, SD_BOTH); ret != 0) {
        // TODO error handling
    }
    if(auto ret = closesocket(socket_handle); ret != 0) {
        // TODO error handling
    }
    if(auto ret = WSACleanup(); ret != 0) {
        throw std::system_error(ret, std::generic_category(), "WSACleanup failed!");
    }
#else
    if(auto ret = shutdown(socket_handle, SHUT_RDWR); ret != 0) {
        // TODO error handling
    }
    if(auto ret = close(socket_handle); ret != 0) {
        // TODO error handling
    }
#endif
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

bool RJRobot::isValidSocketHandle(const socket_t &socket) {
#ifdef _WIN32
    return socket == INVALID_SOCKET;
#else
    return socket >= 0;
#endif
}
