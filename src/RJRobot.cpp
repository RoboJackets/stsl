#include <iostream>
#include <numeric>
#include <cstdlib>
#include <thread>

#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/motor.h>
#include <rc/encoder_eqep.h>

#include <STSL/RJRobot.h>

using namespace std;

RJRobot::RJRobot() {
    std::fill(encoder_buffer_left.begin(), encoder_buffer_left.end(), 0);
    std::fill(encoder_buffer_right.begin(), encoder_buffer_right.end(), 0);
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
    camera.open(0);
    if(!camera.isOpened()) {
        std::cerr << "ERROR: Failed to initialize camera\n";
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
    if(rc_mpu_power_off()) {
        std::cerr << "ERROR: Failed to power down MPU\n";
        exit(EXIT_FAILURE);
    }
    encoder_thread_exit_signal.set_value();
    encoder_monitor_thread.join();
    if(rc_encoder_eqep_cleanup()) {
        std::cerr << "ERROR: Failed to cleanup eQEP encoders\n";
        exit(EXIT_FAILURE);
    }
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

RJRobot::EncoderSpeeds RJRobot::getEncoderSpeeds() {
    static const double buffer_duration = (MS_PER_ENCODER_SAMPLE * ENCODER_BUFFER_SIZE)  / 1000.0;

    auto left_speed = (std::accumulate(encoder_buffer_left.begin(), encoder_buffer_left.end(), 0) * M_PER_ENCODER_TICK) / buffer_duration;
    auto right_speed = (std::accumulate(encoder_buffer_right.begin(), encoder_buffer_right.end(), 0) * M_PER_ENCODER_TICK) / buffer_duration;

    return {left_speed, right_speed};
}

void RJRobot::wait(std::chrono::microseconds duration) {
    std::this_thread::sleep_for(duration);
}

void RJRobot::checkForBattery() {
    battery_found = (rc_adc_batt() >= 6.0) && !(rc_adc_dc_jack() >= 1.0);
    if(!battery_found) {
        std::cerr << "WARNING: Battery is either low, disconnected, or being charged on the DC jack. Motors and servos disabled.\n";
    }
}

void RJRobot::encoderMonitorWorker(std::future<void> exitFuture) {
    size_t buffer_index = 0;
    int last_encoder_position_left = 0;
    int last_encoder_position_right = 0;
    rc_encoder_eqep_write(LEFT_ENCODER_CHANNEL, 0);
    rc_encoder_eqep_write(RIGHT_ENCODER_CHANNEL, 0);
    while (exitFuture.wait_for(std::chrono::milliseconds(MS_PER_ENCODER_SAMPLE)) ==
           std::future_status::timeout) {
        auto left_position = rc_encoder_eqep_read(LEFT_ENCODER_CHANNEL);
        auto right_position = rc_encoder_eqep_read(RIGHT_ENCODER_CHANNEL);

        encoder_buffer_left[buffer_index] =
            left_position - last_encoder_position_left;
        encoder_buffer_right[buffer_index] =
            right_position - last_encoder_position_right;

        last_encoder_position_left = left_position;
        last_encoder_position_right = right_position;

        if (last_encoder_position_left >= ENCODER_POS_ROLLOVER_THRESHOLD
            || last_encoder_position_left <= -ENCODER_POS_ROLLOVER_THRESHOLD) {
            rc_encoder_eqep_write(LEFT_ENCODER_CHANNEL, 0);
            last_encoder_position_left = 0;
        }
        if (last_encoder_position_right >= ENCODER_POS_ROLLOVER_THRESHOLD
            || last_encoder_position_right <= -ENCODER_POS_ROLLOVER_THRESHOLD) {
            rc_encoder_eqep_write(RIGHT_ENCODER_CHANNEL, 0);
            last_encoder_position_right = 0;
        }

        buffer_index = (buffer_index + 1) % ENCODER_BUFFER_SIZE;
    }
}

cv::Mat RJRobot::getImage() {
    cv::Mat frame;
    camera.read(frame);
    if(frame.empty()) {
        std::cerr << "ERROR: Camera returned an empty frame!\n";
    }
    return frame;
}

