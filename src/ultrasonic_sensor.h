#ifndef SOFTWARETRAININGSUPPORTLIBRARY_ULTRASONIC_SENSOR_H
#define SOFTWARETRAININGSUPPORTLIBRARY_ULTRASONIC_SENSOR_H

#include <thread>
#include <atomic>

#include <gpiod.hpp>

class UltrasonicSensor {
public:
  UltrasonicSensor(const std::string& trigger_line_name, const std::string& echo_line_name);

  float getDistance();

private:
  gpiod::line trigger_line_;
  gpiod::line echo_line_;

};


#endif //SOFTWARETRAININGSUPPORTLIBRARY_ULTRASONIC_SENSOR_H
