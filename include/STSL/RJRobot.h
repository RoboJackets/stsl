#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

    void setDriveMotors(double leftPower, double rightPower);

    void stopMotors();

    double getBatteryVoltage();

private:

    void checkForBattery();

    bool battery_found = false;

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
