#include "stsl_nav_plugins/pd_controller.hpp"

#include <iostream>

namespace stsl_nav_plugins
{

void PDController::setParameters(const double kP, const double kD)
{
    std::cout << "kP = " << kP << "\n";
    kP_ = kP;
    kD_ = kD;
}

void PDController::reset()
{
    previous_error_ = 0.0;
}

double PDController::step(const double error)
{
    std::cout << "dError = " << (error - previous_error_) << "\n";
    std::cout << "diff term = " << (kD_ * (error - previous_error_)) << "\n";
    std::cout << "error = " << error << "\n";
    std::cout << "kP = " << kP_ << "\n";
    std::cout << "prop term = " << (kP_ * error) << std::endl;
    const double output = (kP_ * error) + (kD_ * (error - previous_error_));
    previous_error_ = error;
    return output;
}

}
