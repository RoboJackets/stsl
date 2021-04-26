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

#ifndef ULTRASONIC_SENSOR_HPP_
#define ULTRASONIC_SENSOR_HPP_

#include <gpiod.hpp>
#include <atomic>
#include <functional>
#include <string>
#include <thread>

class UltrasonicSensor
{
public:
  using Callback_t = std::function<void (float)>;

  UltrasonicSensor(
    const std::string & trigger_line_name, const std::string & echo_line_name,
    Callback_t measurement_callback);

  ~UltrasonicSensor();

  void trigger();

private:
  gpiod::line trigger_line_;
  gpiod::line echo_line_;
  std::atomic_bool interrupted_{false};
  std::thread echo_listener_thread_;
  Callback_t measurement_callback_;

  void echoListenerThreadFunction();
};

#endif  // ULTRASONIC_SENSOR_HPP_
