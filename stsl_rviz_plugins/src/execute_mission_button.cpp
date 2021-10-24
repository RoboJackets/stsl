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

#include "ros_action_button.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <stsl_interfaces/action/execute_mission.hpp>

namespace stsl_rviz_plugins
{

class ExecuteMission : public RosActionButton<stsl_interfaces::action::ExecuteMission>
{
public:
  explicit ExecuteMission(QWidget * parent = nullptr)
    : RosActionButton("/execute_mission", "Execute Mission", "rviz_execute_mission_client", parent)
    {
    }

  virtual ~ExecuteMission() = default;
};

}  // namespace stsl_rviz_plugins

PLUGINLIB_EXPORT_CLASS(stsl_rviz_plugins::ExecuteMission, rviz_common::Panel)
