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

#ifndef ACTION_BUTTON_HPP_
#define ACTION_BUTTON_HPP_

#include <QWidget>
#include <QPushButton>
#include <QState>
#include <QStateMachine>
#include <QTimer>
#include <rviz_common/panel.hpp>
#include <string>

namespace stsl_rviz_plugins
{

class ActionButton : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ActionButton(const std::string & button_text, QWidget * parent = nullptr);

  virtual ~ActionButton() = default;

private Q_SLOTS:
  void onEnterRunning();
  void onEnterCancelling();

Q_SIGNALS:
  void cancellingCompleted();
  void runningCompleted();

protected:
  enum class ActionStatus
  {
    Running,
    Aborted,
    Cancelled,
    Completed
  };

private:
  QTimer * timer_;
  QMetaObject::Connection timer_connection_;
  QPushButton * button_;
  QState * idle_state_;
  QState * running_state_;
  QState * cancelling_state_;
  QStateMachine state_machine_;

  void TimerCallback();

  virtual bool SendGoal() = 0;
  virtual void CancelGoal() = 0;
  virtual ActionStatus CheckActionStatus() = 0;
};

}  // namespace stsl_rviz_plugins

#endif  // ACTION_BUTTON_HPP_
