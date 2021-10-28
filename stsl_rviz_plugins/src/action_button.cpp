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

#include "action_button.hpp"
#include <QVBoxLayout>
#include <string>

namespace stsl_rviz_plugins
{

ActionButton::ActionButton(const std::string & button_text, QWidget * parent)
: rviz_common::Panel(parent),
  timer_(new QTimer(this)),
  button_(new QPushButton),
  idle_state_(new QState),
  running_state_(new QState),
  cancelling_state_(new QState)
{
  button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  QFont button_font = button_->font();
  button_font.setPointSize(20);
  button_->setFont(button_font);

  idle_state_->setObjectName("idle");
  idle_state_->assignProperty(button_, "text", button_text.c_str());
  idle_state_->assignProperty(button_, "enabled", true);
  idle_state_->addTransition(button_, SIGNAL(clicked()), running_state_);

  running_state_->setObjectName("running");
  running_state_->assignProperty(button_, "text", "Cancel");
  running_state_->assignProperty(button_, "enabled", true);
  running_state_->addTransition(button_, SIGNAL(clicked()), cancelling_state_);
  running_state_->addTransition(this, SIGNAL(runningCompleted()), idle_state_);
  QObject::connect(running_state_, SIGNAL(entered()), this, SLOT(onEnterRunning()));

  cancelling_state_->setObjectName("cancelling");
  cancelling_state_->assignProperty(button_, "text", "Cancelling...");
  cancelling_state_->assignProperty(button_, "enabled", false);
  cancelling_state_->addTransition(this, SIGNAL(cancellingCompleted()), idle_state_);
  QObject::connect(cancelling_state_, SIGNAL(entered()), this, SLOT(onEnterCancelling()));

  state_machine_.addState(idle_state_);
  state_machine_.addState(running_state_);
  state_machine_.addState(cancelling_state_);
  state_machine_.setInitialState(idle_state_);
  state_machine_.start();

  QVBoxLayout * layout = new QVBoxLayout;
  layout->addWidget(button_);
  setLayout(layout);
}

void ActionButton::onEnterRunning()
{
  if (!SendGoal()) {
    runningCompleted();
    return;
  }
  timer_connection_ = timer_->callOnTimeout(this, &ActionButton::TimerCallback);
  timer_->start(100);
}

void ActionButton::onEnterCancelling()
{
  CancelGoal();
}

void ActionButton::TimerCallback()
{
  switch (CheckActionStatus()) {
    case ActionStatus::Running:
      return;
    case ActionStatus::Aborted:
      runningCompleted();
      break;
    case ActionStatus::Cancelled:
      cancellingCompleted();
      break;
    case ActionStatus::Completed:
      runningCompleted();
      break;
  }
  timer_->stop();
}

}  // namespace stsl_rviz_plugins
