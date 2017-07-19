#include <iostream>
#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot;

    while(true) {
        cout << "\r" << robot.IsButtonPressed() << endl;
        robot.Wait(500ms);
    }

    return 0;
}