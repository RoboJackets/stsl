#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <rc/mpu.h>
#include <thread>
#include <future>

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

private:

    void checkForBattery();

    void encoderMonitorWorker(std::future<void> exitFuture);

    bool battery_found = false;

    rc_mpu_data_t mpu_data = {};

    std::thread encoder_monitor_thread;
    std::promise<void> encoder_thread_exit_signal;

    static const int CENTER_LINE_SENSOR_CHANNEL = 3;
    static const int OFFSET_LINE_SENSOR_CHANNEL = 4;
    static const int LEFT_MOTOR_CHANNEL = 1;
    static const int RIGHT_MOTOR_CHANNEL = 2;

};

#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
