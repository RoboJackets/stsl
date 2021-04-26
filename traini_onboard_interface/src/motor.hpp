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

#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <gpiod.hpp>
#include <fstream>
#include <string>

struct MotorParameters
{
  std::string direction_line_1_name;
  std::string direction_line_2_name;
  std::string pwm_pin_name;
  std::string pwm_chip_device;
  std::string pwm_chip_address;
  std::string pwm_chip_number;
  std::string pwm_index;
};

extern const MotorParameters left_motor_parameters;
extern const MotorParameters right_motor_parameters;

class Motor
{
public:
  explicit Motor(const MotorParameters & params);

  ~Motor();

  /**
   *
   * @param power -1.0 to 1.0
   */
  void setPower(float power);

private:
  gpiod::line direction_line_1_;
  gpiod::line direction_line_2_;
  std::ofstream duty_cycle_file_;
  std::ofstream enable_file_;

  static constexpr int period_{500'000};
};

#endif  // MOTOR_HPP_
