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
#include <cstring>

#endif

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

void RJRobot::SetOnBoardLED(bool on) {
    if(on) {
        sendCommand("SetOnBoardLEDOn\n");
    } else {
        sendCommand("SetOnBoardLEDOff\n");
    }
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
