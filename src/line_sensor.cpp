#include "line_sensor.h"

LineSensor::LineSensor(int input_number)
    : file("/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(input_number) + "_raw")
{
}

int LineSensor::getValue() {
    file.seekg(0);
    int value;
    file >> value;
    return value;
}
