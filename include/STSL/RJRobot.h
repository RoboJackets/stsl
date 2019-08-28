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

private:

    void checkForBattery();

    bool battery_found = false;

    rc_mpu_data_t mpu_data = {};

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
