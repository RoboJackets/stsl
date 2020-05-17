#ifndef SOFTWARETRAININGSUPPORTLIBRARY_MOTOR_H
#define SOFTWARETRAININGSUPPORTLIBRARY_MOTOR_H

#include <fstream>
#include <gpiod.hpp>

struct MotorParameters {
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

class Motor {
public:
  Motor(const MotorParameters &params);

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


#endif //SOFTWARETRAININGSUPPORTLIBRARY_MOTOR_H
