#include <iostream>
#include <stdlib.h>

#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/motor.h>
#include <rc/encoder_eqep.h>

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
    auto mpu_config = rc_mpu_default_config();
    if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
        std::cerr << "ERROR: Failed to initialize MPU\n";
        exit(EXIT_FAILURE);
    }
    if(rc_encoder_eqep_init()) {
        std::cerr << "ERROR: Failed to initialize eQEP encoders\n";
        exit(EXIT_FAILURE);
    }
    encoder_monitor_thread = std::thread(&RJRobot::encoderMonitorWorker, this, std::move(encoder_thread_exit_signal.get_future()));
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
    if(rc_mpu_power_off()) {
        std::cerr << "ERROR: Failed to power down MPU\n";
        exit(EXIT_FAILURE);
    }
    if(rc_encoder_eqep_cleanup()) {
        std::cerr << "ERROR: Failed to cleanup eQEP encoders\n";
        exit(EXIT_FAILURE);
    }
    encoder_thread_exit_signal.set_value();
    encoder_monitor_thread.join();
}

void RJRobot::setDriveMotors(double left_power, double right_power) {
    if(!battery_found) return;
    if(rc_motor_set(LEFT_MOTOR_CHANNEL, left_power)) {
        std::cerr << "ERROR: Failed to set left motor power\n";
        exit(EXIT_FAILURE);
    }
    if(rc_motor_set(RIGHT_MOTOR_CHANNEL, right_power)) {
        std::cerr << "ERROR: Failed to set right motor power\n";
        exit(EXIT_FAILURE);
    }
}

void RJRobot::stopMotors() {
    if(!battery_found) return;
    if(rc_motor_free_spin(LEFT_MOTOR_CHANNEL)) {
        std::cerr << "ERROR: Failed to free spin left motor\n";
        exit(EXIT_FAILURE);
    }
    if(rc_motor_free_spin(RIGHT_MOTOR_CHANNEL)) {
        std::cerr << "ERROR: Failed to free spin right motor\n";
        exit(EXIT_FAILURE);
    }
}

double RJRobot::getBatteryVoltage() {
    return rc_adc_batt();
}

const rc_mpu_data_t &RJRobot::getMPUData() {
    return mpu_data;
}

double RJRobot::getCenterLineSensor() {
    return rc_adc_read_volt(CENTER_LINE_SENSOR_CHANNEL);
}

double RJRobot::getOffsetLineSensor() {
    return rc_adc_read_volt(OFFSET_LINE_SENSOR_CHANNEL);
}

void RJRobot::checkForBattery() {
    battery_found = (rc_adc_batt() >= 6.0) && !(rc_adc_dc_jack() >= 1.0);
    if(!battery_found) {
        std::cerr << "ERROR: Battery is either low, disconnected, or being charged on the DC jack. Motors and servos disabled.\n";
    }
}

void RJRobot::encoderMonitorWorker(std::future<void> exitFuture) {
    while(exitFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
        std::cout << "looping\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Thread ended!\n";
}
