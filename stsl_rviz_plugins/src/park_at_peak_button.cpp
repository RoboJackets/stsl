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

#include <stsl_interfaces/action/park_at_peak.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "ros_action_button.hpp"

namespace stsl_rviz_plugins
{

class ParkAtPeakButton : public RosActionButton<stsl_interfaces::action::ParkAtPeak>
{
public:
  explicit ParkAtPeakButton(QWidget * parent = nullptr)
  : RosActionButton("/park_at_peak", "Park at Peak", "rviz_park_at_peak_client", parent)
  {
  }

  virtual ~ParkAtPeakButton() = default;
};

}  // namespace stsl_rviz_plugins

PLUGINLIB_EXPORT_CLASS(stsl_rviz_plugins::ParkAtPeakButton, rviz_common::Panel)
