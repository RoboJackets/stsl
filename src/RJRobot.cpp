#include <iostream>
#include <stdlib.h>

#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/motor.h>

#include <STSL/RJRobot.h>

using namespace std;

RJRobot::RJRobot() {
    if(rc_adc_init()) {
        std::cerr << "ERROR: Failed to run rc_adc_init()\n";
        exit(EXIT_FAILURE);
    }
    checkForBattery();
    if(battery_found) {
        if(rc_motor_init()) {
            std::cerr << "ERROR: Failed to intialize motors\n";
	    exit(EXIT_FAILURE);
        }
    }
}

RJRobot::~RJRobot() {
    if(battery_found) {
        if(rc_motor_cleanup()) {
            std::cerr << "ERROR: Failed to cleanup motors\n";
        }
    }
    if(rc_adc_cleanup()) {
        std::cerr << "ERROR: Failed to run rc_adc_cleanup()\n";
        exit(EXIT_FAILURE);
    }
}

void RJRobot::setDriveMotors(double left_power, double right_power) {
    if(!battery_found) return;
    if(rc_motor_set(1, left_power)) exit(-1);
    if(rc_motor_set(2, right_power)) exit(-1);
}

void RJRobot::stopMotors() {
    if(!battery_found) return;
    if(rc_motor_free_spin(1)) exit(-1);
    if(rc_motor_free_spin(2)) exit(-1);
}

double RJRobot::getBatteryLevel() {
    return rc_adc_batt();
}

void RJRobot::checkForBattery() {
    battery_found = rc_adc_batt() >= 6.0;
    if(!battery_found) {
        std::cerr << "ERROR: Battery not found (or undercharged). Motors and servos disabled.\n";
    }
}

