#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <rc/mpu.h>

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

    void setDriveMotors(double leftPower, double rightPower);

    void stopMotors();

    double getBatteryVoltage();

    const rc_mpu_data_t &getMPUData();

    double getCenterLineSensor();

    double getOffsetLineSensor();

private:

    void checkForBattery();

    bool battery_found = false;

    rc_mpu_data_t mpu_data = {};

    static const int CENTER_LINE_SENSOR_CHANNEL = 3;
    static const int OFFSET_LINE_SENSOR_CHANNEL = 4;
    static const int LEFT_MOTOR_CHANNEL = 1;
    static const int RIGHT_MOTOR_CHANNEL = 2;

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
