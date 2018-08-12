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

enum class LightSensor {
    CENTER,
    RIGHT
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

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

    void SetMotor(const Motor &port, const int &speed);

    void StopMotors();

    int GetLightValue(const LightSensor &sensor);

    double GetUltrasonicDistance();

    double GetProximity();

    Gesture GetGesture();

    Color GetColor();

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
