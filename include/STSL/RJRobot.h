#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <memory>
#include <chrono>

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

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
