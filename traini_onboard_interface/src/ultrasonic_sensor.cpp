// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "src/ultrasonic_sensor.hpp"
#include <string>

using namespace std::chrono_literals;

UltrasonicSensor::UltrasonicSensor(
  const std::string & trigger_line_name,
  const std::string & echo_line_name,
  UltrasonicSensor::Callback_t measurement_callback)
: measurement_callback_(measurement_callback)
{
  trigger_line_ = gpiod::find_line(trigger_line_name);
  if (!trigger_line_) {
    throw std::logic_error("Could not find line with name: " + trigger_line_name);
  }
  trigger_line_.request({"robot_interface_node", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
  if (!trigger_line_.is_requested()) {
    throw std::logic_error("Could not take ownership of " + trigger_line_name);
  }

  echo_line_ = gpiod::find_line(echo_line_name);
  if (!echo_line_) {
    throw std::logic_error("Could not find line with name: " + echo_line_name);
  }
  echo_line_.request({"robot_interface_node", gpiod::line_request::EVENT_BOTH_EDGES, 0});
  if (!echo_line_.is_requested()) {
    throw std::logic_error("Could not take ownership of " + echo_line_name);
  }

  echo_listener_thread_ = std::thread{[this]() {echoListenerThreadFunction();}};
}

UltrasonicSensor::~UltrasonicSensor()
{
  interrupted_ = true;
  echo_listener_thread_.join();
}

void UltrasonicSensor::trigger()
{
  trigger_line_.set_value(1);
  std::this_thread::sleep_for(10us);
  trigger_line_.set_value(0);
}

void UltrasonicSensor::echoListenerThreadFunction()
{
  std::chrono::nanoseconds rise_time;
  while (!interrupted_) {
    if (echo_line_.event_wait(std::chrono::seconds(1))) {
      const auto & event = echo_line_.event_read();
      if (event.event_type == gpiod::line_event::RISING_EDGE) {
        rise_time = event.timestamp;
      } else {
        const auto elapsed_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(event.timestamp - rise_time)
          .count();
        measurement_callback_(elapsed_time * 170.0f);
      }
    }
  }
}
