#include <iostream>
#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot;

    bool light = true;

    while(true) {
        robot.SetOnBoardLED(light);
        light = !light;
        robot.Wait(1000ms);
    }

    return 0;
}