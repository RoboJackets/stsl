#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <memory>
#include <chrono>

#ifdef _WIN32
using socket_t = unsigned long long;
#else
using socket_t = int;
#endif

enum class MotorPort {
    A,
    B
};

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

    void SetOnBoardLED(bool on);

    void SetMotor(const MotorPort &port, const int &speed);

    void StopMotors();

    void Wait(std::chrono::microseconds duration);

private:

    socket_t socket_handle;

    bool isValidSocketHandle(const socket_t &socket);

    void sendCommand(const char* command);

    void handleError(const char *message);

    void handleError(int retval, const char *message);

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
