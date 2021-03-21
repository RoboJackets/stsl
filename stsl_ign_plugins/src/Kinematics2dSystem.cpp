#include "Kinematics2dSystem.h"
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>

namespace stsl_ign_plugins::systems
{

void Kinematics2dSystem::Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr) 
{
    this->entity = entity;

    auto model = ignition::gazebo::Model(entity);
    
    if(!model.Valid(ecm))
    {
        ignerr << "Kinematics2d plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }

    std::string topic = "/model/" + model.Name(ecm) + "/cmd_vel";

    if(!node.Subscribe(topic, &Kinematics2dSystem::OnCmdVel, this))
    {
        ignerr << "Could not subscribe to topic: " << std::endl;
        return;
    }

    ignmsg << "Listening for commanded velocities on [" << topic << "]" << std::endl;
}

void Kinematics2dSystem::PreUpdate(const ignition::gazebo::UpdateInfo& info, 
                   ignition::gazebo::EntityComponentManager& ecm)
{
    if(info.paused)
    {
        return;
    }

    const auto pose = ecm.ComponentData<ignition::gazebo::components::Pose>(entity);

    if(!pose)
    {
        ignerr << "Could not get link pose." << std::endl;
        return;
    }

    const auto next_pose = getNextPose(pose.value(), info.dt);

    ecm.SetComponentData<ignition::gazebo::components::Pose>(entity, next_pose);
    ecm.SetChanged(entity, ignition::gazebo::components::Pose::typeId, ignition::gazebo::ComponentState::PeriodicChange);
}

void Kinematics2dSystem::OnCmdVel(const ignition::msgs::Twist &msg)
{
    std::lock_guard<std::mutex> lock(target_velocity_mutex);
    target_velocity = msg;
}

ignition::math::Pose3d Kinematics2dSystem::getNextPose(const ignition::math::Pose3d& pose, const std::chrono::duration<double>& elapsed)
{
    const auto [linear_velocity, angular_velocity] = [=]{
        std::lock_guard<std::mutex> lock(target_velocity_mutex);
        return std::make_pair(target_velocity.linear().x(), target_velocity.angular().z());
    }();

    double dx = 0.0;
    double dy = 0.0;
    double dyaw = 0.0;

    if(std::abs(angular_velocity) < 1e-8)
    {
        dx = linear_velocity*elapsed.count();
    }
    else 
    {
        const auto s = linear_velocity * elapsed.count();
        dyaw = angular_velocity * elapsed.count();
        const auto r = s / dyaw;
        const auto yaw_prime = pose.Yaw() + dyaw;
        dx = r * (std::cos(yaw_prime) - std::cos(pose.Yaw()));
        dy = r * (std::sin(yaw_prime) - std::sin(pose.Yaw()));
    }

    return ignition::math::Pose3d(
        pose.X() + dx, 
        pose.Y() + dy, 
        pose.Z(), 
        pose.Roll(), 
        pose.Pitch(), 
        pose.Yaw() + dyaw);
}

}

IGNITION_ADD_PLUGIN(
    stsl_ign_plugins::systems::Kinematics2dSystem,
    ignition::gazebo::System,
    stsl_ign_plugins::systems::Kinematics2dSystem::ISystemConfigure,
    stsl_ign_plugins::systems::Kinematics2dSystem::ISystemPreUpdate
)
