#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <memory>
#include <chrono>

#ifdef _WIN32
using socket_t = unsigned long long;
#else
using socket_t = int;
#endif

enum class Motor {
    LEFT,
    RIGHT,
    LIFT
};

enum class LineSensor {
    CENTER,
    OFFSET
};

enum class Color {
    RED,
    BLUE,
    UNKNOWN
};

enum class Gesture {
    DOWN,
    UP,
    LEFT,
    RIGHT,
    NONE
};

enum RobotType {
    REAL,
    SIMULATOR
};

class RJRobot {

public:
    RJRobot(RobotType type);

    ~RJRobot();

    void SetMotor(const Motor &port, const int &speed);

    void StopMotors();

    double GetUltrasonicDistance();

    double GetProximity();

    Gesture GetGesture();

    Color GetColor();

    int GetLineValue(const LineSensor &sensor);

    void Wait(std::chrono::microseconds duration);

private:

    socket_t socket_handle;

    bool isValidSocketHandle(const socket_t &socket);

    void sendCommand(const char* command);

    std::string getResponse();

    void handleError(const char *message);

    void handleError(int retval, const char *message);

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
