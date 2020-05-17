#include "motor.h"
#include <algorithm>
#include <iostream>

const MotorParameters left_motor_parameters{
    "GPMC_WAIT0",  // P9_11
    "GPMC_WPN",  // P9_13
    "P9_14",
    "48302000",
    "48302200",
    "4",
    "0"
};

const MotorParameters right_motor_parameters{
    "GPMC_A0",  // P9_15
    "SPI0_CS0",  // P9_17
    "P9_16",
    "48302000",
    "48302200",
    "4",
    "1"
};

Motor::Motor(const MotorParameters &params) {
    direction_line_1_ = gpiod::find_line(params.direction_line_1_name);
    if(!direction_line_1_)
    {
        throw std::logic_error("Could not find line with name: " + params.direction_line_1_name);
    }
    direction_line_2_ = gpiod::find_line(params.direction_line_2_name);
    if(!direction_line_2_)
    {
        throw std::logic_error("Could not find line with name: " + params.direction_line_2_name);
    }

    direction_line_1_.request({"robot_interface_node", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
    if(!direction_line_1_.is_requested())
    {
        throw std::logic_error("Could not take ownership of " + params.direction_line_1_name);
    }

    direction_line_2_.request({"robot_interface_node", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
    if(!direction_line_2_.is_requested())
    {
        throw std::logic_error("Could not take ownership of " + params.direction_line_2_name);
    }

    std::ofstream pinmux_file{"/sys/devices/platform/ocp/ocp:" + params.pwm_pin_name + "_pinmux/state"};
    pinmux_file.seekp(0);
    pinmux_file << "pwm";
    pinmux_file.close();

    std::ofstream export_file{"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip4/export"};
    export_file.seekp(0);
    export_file << "0";
    export_file.close();

    const auto base_path = "/sys/devices/platform/ocp/" + params.pwm_chip_device + ".epwmss/" + params.pwm_chip_address + ".pwm/pwm/pwmchip" + params.pwm_chip_number + "/pwm-" + params.pwm_chip_number + ":" + params.pwm_index + "/";

    std::ofstream period_file{base_path + "period"};
    period_file.seekp(0);
    period_file << std::to_string(period_);
    period_file.close();

    std::ofstream polarity_file{base_path + "polarity"};
    polarity_file.seekp(0);
    polarity_file << "normal";
    polarity_file.close();

    duty_cycle_file_ = std::ofstream{base_path + "duty_cycle"};
    duty_cycle_file_.seekp(0);
    duty_cycle_file_ << "0";
    duty_cycle_file_.flush();

    enable_file_ = std::ofstream {base_path + "enable"};
    enable_file_.seekp(0);
    enable_file_ << "1";
    enable_file_.flush();
}

Motor::~Motor() {
    enable_file_.seekp(0);
    enable_file_ << "0";
}

void Motor::setPower(float power) {
    if(power < 0)
    {
        power *= -1;
        direction_line_1_.set_value(0);
        direction_line_2_.set_value(1);
    }
    else
    {
        direction_line_1_.set_value(1);
        direction_line_2_.set_value(0);
    }
    power = std::clamp(power, 0.0f, 1.0f);
    duty_cycle_file_.seekp(0);
    duty_cycle_file_ << std::to_string(power * period_);
    duty_cycle_file_.flush();
}
