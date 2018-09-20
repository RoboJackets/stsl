#include <iostream>
#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot(REAL);

    while(true) {
        robot.Wait(1000ms);
        robot.SetDriveMotors(255, 255);
        robot.Wait(2000ms);
        robot.StopMotors();
    }

    return 0;
}