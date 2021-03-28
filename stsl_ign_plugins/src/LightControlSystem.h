#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <atomic>

namespace stsl_ign_plugins::systems
{

class LightControlSystem
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
    void OnControlMessage(const ignition::msgs::Boolean& msg);

    ignition::transport::Node node;

    ignition::gazebo::Entity entity;

    double light_range{0.0};

    std::mutex state_mutex;
    bool changed{false};
    bool light_is_on{true};

};

}