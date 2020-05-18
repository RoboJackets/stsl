#ifndef SOFTWARETRAININGSUPPORTLIBRARY_ULTRASONIC_SENSOR_H
#define SOFTWARETRAININGSUPPORTLIBRARY_ULTRASONIC_SENSOR_H

#include <thread>
#include <atomic>
#include <functional>

#include <gpiod.hpp>

class UltrasonicSensor {
public:
  using Callback_t = std::function<void(float)>;

  UltrasonicSensor(const std::string& trigger_line_name, const std::string& echo_line_name, Callback_t measurement_callback);

  void trigger();

private:
  gpiod::line trigger_line_;
  gpiod::line echo_line_;
  std::atomic_bool interrupted_{false};
  std::thread echo_listener_thread_;
  Callback_t measurement_callback_;

  void echoListenerThreadFunction();

};


#endif //SOFTWARETRAININGSUPPORTLIBRARY_ULTRASONIC_SENSOR_H
