#include <unistd.h>
#include <STSL/RJRobot.h>

using namespace std;

int main() {
    RJRobot robot;

    robot.setDriveMotors(1.0, 1.0);

    usleep(2'000'000);

    robot.stopMotors();

    return 0;
}
