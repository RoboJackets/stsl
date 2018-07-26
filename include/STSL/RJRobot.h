#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <memory>
#include <chrono>

#ifdef _WIN32
using socket_t = SOCKET;
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

    bool IsButtonPressed();

    uint8_t LightValue();

    void SetFloodlight(bool on);

    void SetMotor(const MotorPort &port, const int &speed);

    void StopMotors();

    void Wait(std::chrono::microseconds duration);

private:

    socket_t socket_handle;

    bool isValidSocketHandle(const socket_t &socket);

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
