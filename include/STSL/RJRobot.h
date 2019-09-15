#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <rc/mpu.h>
#include <thread>
#include <future>
#include <cmath>
#include <chrono>

using namespace std::literals::chrono_literals;

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

    /**
     * Sets the speed of the drive motors by controlling their PWM duty cycle
     * @param leftPower Left motor duty cycle (-1.0 full reverse, 1.0 full forward)
     * @param rightPower Right motor duty cycle (-1.0 full reverse, 1.0 full forward)
     */
    void setDriveMotors(double leftPower, double rightPower);

    /**
     * Sets both drive motors to free spin, allowing the robot to coast to a stop.
     */
    void stopMotors();

    /**
     * Reads the battery pack voltage via the onboard ADC.
     * @return The voltage of the battery pack
     */
    double getBatteryVoltage();

    /**
     * Reads the IMU / MPU data from the built in sensors.
     * @return A <a href="http://strawsondesign.com/docs/librobotcontrol/structrc__mpu__data__t.html">rc_mpu_data_t</a> struct containing the latest sensor data.
     */
    const rc_mpu_data_t &getMPUData();

    /**
     * Reads the output of the center line sensor
     * @return The voltage read from the center line sensor
     */
    double getCenterLineSensor();

    /**
     * Reads the output of the offset line sensor
     * @return The voltage read from the offset line sensor
     */
    double getOffsetLineSensor();

    struct EncoderSpeeds {
      double left;
      double right;
    };

    /**
     * Calculates the wheel ground speeds from the encoder buffer being maintained in the background.
     * @return An EncoderSpeeds struct instance filled with the wheel speeds in m/s
     */
    EncoderSpeeds getEncoderSpeeds();

    void wait(std::chrono::microseconds duration);

private:

    void checkForBattery();

    void encoderMonitorWorker(std::future<void> exitFuture);

    bool battery_found = false;

    rc_mpu_data_t mpu_data = {};

    std::thread encoder_monitor_thread;
    std::promise<void> encoder_thread_exit_signal;

    const int64_t MS_PER_ENCODER_SAMPLE = 10;
    static const size_t ENCODER_BUFFER_SIZE = 10;
    static const int ENCODER_POS_ROLLOVER_THRESHOLD = 536870912; // (2^29)
    static constexpr double M_PER_ENCODER_TICK = (M_PI * 0.065 /*wheel diameter (m)*/)  / 40 /* ticks per rev */;
    std::array<int, ENCODER_BUFFER_SIZE> encoder_buffer_left;
    std::array<int, ENCODER_BUFFER_SIZE> encoder_buffer_right;

    static const int CENTER_LINE_SENSOR_CHANNEL = 3;
    static const int OFFSET_LINE_SENSOR_CHANNEL = 4;
    static const int LEFT_MOTOR_CHANNEL = 1;
    static const int RIGHT_MOTOR_CHANNEL = 2;
    static const int LEFT_ENCODER_CHANNEL = 1;
    static const int RIGHT_ENCODER_CHANNEL = 2;

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
