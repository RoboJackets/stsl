#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <mutex>

namespace ign_kinematics_system
{

class IgnKinematicsSystem
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
    void Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info, 
                   ignition::gazebo::EntityComponentManager& ecm) override;

private:
    void OnCmdVel(const ignition::msgs::Twist &msg);

    ignition::math::Pose3d getNextPose(const ignition::math::Pose3d& pose, const std::chrono::duration<double>& elapsed);

    ignition::transport::Node node;

    std::mutex target_velocity_mutex;
    ignition::msgs::Twist target_velocity;

    ignition::gazebo::Entity entity;
};

}
