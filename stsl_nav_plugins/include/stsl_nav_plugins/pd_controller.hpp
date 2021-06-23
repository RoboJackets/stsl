#ifndef STSL_NAV_PLUGINS_PD_CONTROLLER_HPP
#define STSL_NAV_PLUGINS_PD_CONTROLLER_HPP

namespace stsl_nav_plugins
{
class PDController
{
public:
  void setParameters(const double kP, const double kD);
  void reset();
  double step(const double error);

private:
  double kP_;
  double kD_;
  double previous_error_;
};

}  // namespace stsl_nav_plugins

#endif
