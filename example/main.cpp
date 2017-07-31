#include <iostream>
#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot;

    bool floodlight = true;

    while(true) {
        robot.SetFloodlight(floodlight);
        floodlight = !floodlight;
        cout << "\r" << robot.IsButtonPressed() << "\t" << static_cast<int>(robot.LightValue()) << endl;
        robot.Wait(500ms);
    }

    return 0;
}