#include "stsl_nav_plugins/pd_controller.hpp"

namespace stsl_nav_plugins
{

void PDController::setParameters(const double kP, const double kD)
{
    kP_ = kP;
    kD_ = kD;
}

void PDController::reset()
{
    previous_error_ = 0.0;
}

double PDController::step(const double error)
{
    const double output = (kP_ * error) + (kD_ * (error - previous_error_));
    previous_error_ = error;
    return output;
}

}
