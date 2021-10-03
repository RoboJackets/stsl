#include <QWidget>
#include <QPushButton>
#include <QState>
#include <QStateMachine>
#include <QTimer>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stsl_interfaces/action/park_at_peak.hpp>

namespace stsl_rviz_plugins
{

class ParkAtPeakButton : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit ParkAtPeakButton(QWidget * parent = nullptr);
  virtual ~ParkAtPeakButton() = default;

private Q_SLOTS:
  void onEnterRunning();
  void onEnterCancelling();

Q_SIGNALS:
  void cancellingCompleted();
  void runningCompleted();

private:
  const std::chrono::seconds server_timeout_{1};
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp_action::Client<stsl_interfaces::action::ParkAtPeak>::SharedPtr action_client_;
  using GoalHandle = rclcpp_action::ClientGoalHandle<stsl_interfaces::action::ParkAtPeak>;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> action_result_future_;

  QTimer * timer_;
  QMetaObject::Connection timer_connection_;

  QPushButton * button_;

  QState * idle_state_;
  QState * running_state_;
  QState * cancelling_state_;
  QStateMachine state_machine_;

  void CheckActionStatus();
};

}
