#include <iostream>
#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot(REAL);

    while(true) {

        robot.Wait(1000ms);
        robot.SetMotor(Motor::LEFT, -255);
        robot.SetMotor(Motor::RIGHT, 255);
        robot.Wait(2000ms);
        robot.StopMotors();
    }

    return 0;
}