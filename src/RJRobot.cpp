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
#include <netinet/tcp.h>

using namespace std;

RJRobot::RJRobot(RobotType type) {
    cout << "Initializing..." << endl;

#ifdef _WIN32
    WSADATA wsa_data;
    if(auto ret = WSAStartup(MAKEWORD(1,1), &wsa_data); ret != 0) {
        handleError(ret, "WSAStartup failed");
    }
#endif

    const char* HUZZAH_IP;

    if(type == REAL) {
        HUZZAH_IP = "10.10.10.1";
        cout << "Looking for a real robot." << endl;
    } else {
        HUZZAH_IP = "127.0.0.1";
        cout << "Looking for the simulator." << endl;
    }

    socket_handle = socket(AF_INET, SOCK_STREAM, 0);

    if(!isValidSocketHandle(socket_handle)) {
        handleError("Unable to create socket");
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8008);

    if(auto ret = inet_pton(AF_INET, HUZZAH_IP, &serv_addr.sin_addr); ret != 1) {
        handleError("inet_pton failed");
    }

    if(auto ret = connect(socket_handle, reinterpret_cast<sockaddr*>(&serv_addr), sizeof(serv_addr)); ret != 0) {
        handleError("connect failed");
    }

    // TODO Why does this make it work? Learn something please.
    int one = 1;
    setsockopt(socket_handle, SOL_TCP, TCP_NODELAY, &one, sizeof(one));

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

void RJRobot::SetDriveMotors(int leftPower, int rightPower) {
    // leftPower is negated so that positive powers result in forward motion on both drive motors
    sendCommand(("SetDrive" + to_string(-1*leftPower) + "|" + to_string(rightPower) + "\n").c_str());
}

void RJRobot::SetLiftMotor(int power) {
    sendCommand(("SetLift" + to_string(power) + "\n").c_str());
}

void RJRobot::StopMotors() {
    sendCommand("StopMotors\n");
}

double RJRobot::GetUltrasonicDistance() {
    sendCommand("GetUltrasonic\n");

    auto response = getResponse();

    return std::stod(response);
}

double RJRobot::GetProximity() {
    sendCommand("GetProximity\n");
    auto response = getResponse();
    return std::stod(response);
}

Gesture RJRobot::GetGesture() {
    sendCommand("GetGesture\n");
    auto response = getResponse();
    if(response == "UP") return Gesture::UP;
    if(response == "DOWN") return Gesture::DOWN;
    if(response == "LEFT") return Gesture::LEFT;
    if(response == "RIGHT") return Gesture::RIGHT;
    return Gesture::NONE;
}

Color RJRobot::GetColor() {
    sendCommand("GetColor\n");
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

int RJRobot::GetLineValue(const LineSensor &sensor) {
    sendCommand(sensor == LineSensor::CENTER ? "GetLineCenter\n" : "GetLineOffset\n");
    auto response = getResponse();
    return std::stoi(response);
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
