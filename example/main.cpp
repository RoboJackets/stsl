#include <iostream>
#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot;

    bool floodlight = true;

    int speed = 255;

    while(true) {
        robot.SetFloodlight(floodlight);
        floodlight = !floodlight;
        robot.SetMotor(MotorPort::A, speed);
        speed = (speed == 255 ? 0 : 255);
        cout << "\r" << robot.IsButtonPressed() << "\t" << static_cast<int>(robot.LightValue()) << endl;
        robot.Wait(500ms);
    }

    return 0;
}