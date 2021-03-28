#include "LightControlSystem.h"
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Name.hh>

namespace stsl_ign_plugins::systems
{

void LightControlSystem::Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr)
{
    auto sdf_ptr = const_cast<sdf::Element *>(sdf.get());

    if(!sdf_ptr->HasElement("light_name"))
    {
        ignerr << "A <light_name> tag must be specified." << std::endl;
        return;
    }

    const auto light_name = sdf_ptr->GetElement("light_name")->GetValue()->GetAsString();

    const auto named_entities = ecm.EntitiesByComponents(ignition::gazebo::components::Name(light_name));

    const auto found_entity_iter = std::find_if(named_entities.begin(), named_entities.end(), [&ecm](const auto& e){
        return ecm.EntityHasComponentType(e, ignition::gazebo::components::Light::typeId);
    });

    if(found_entity_iter == named_entities.end()) {
        ignerr << "No lights found with name '" << light_name << "'" << std::endl;
        return;
    }

    this->entity = *found_entity_iter;

    auto component_factory = ignition::gazebo::components::Factory::Instance();
    const auto components = ecm.ComponentTypes(this->entity);
    for(const auto& component_type : components) {
        ignmsg << "Component type: " << component_factory->Name(component_type) << std::endl;
    }

    light_range = ecm.ComponentData<ignition::gazebo::components::Light>(this->entity).value().AttenuationRange();

    std::string topic = "/light/" + light_name;

    if(!node.Subscribe(topic, &LightControlSystem::OnControlMessage, this))
    {
        ignerr << "Could not subscribe to topic: " << std::endl;
        return;
    }

    ignmsg << "Listening for light commands on [" << topic << "]" << std::endl;
}

void LightControlSystem::PreUpdate(const ignition::gazebo::UpdateInfo& info, 
                   ignition::gazebo::EntityComponentManager& ecm) 
{
    auto [is_changed, is_on] = [this]{
        std::lock_guard<std::mutex> lock(state_mutex);
        return std::make_pair(changed, light_is_on);
    }();
    if(!is_changed)
        return;

    auto light_data = ecm.ComponentData<ignition::gazebo::components::Light>(entity).value();
    igndbg << "Light state: " << (is_on ? "ON" : "OFF") << std::endl;
    light_data.SetAttenuationRange(is_on ? light_range : 0.0);
    ecm.SetComponentData<ignition::gazebo::components::Light>(entity, light_data);
    ecm.SetChanged(entity, ignition::gazebo::components::Light::typeId, ignition::gazebo::ComponentState::OneTimeChange);
    changed = false;
}

void LightControlSystem::OnControlMessage(const ignition::msgs::Boolean& msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    light_is_on = msg.data();
    changed = true;
}

}

IGNITION_ADD_PLUGIN(
    stsl_ign_plugins::systems::LightControlSystem,
    ignition::gazebo::System,
    stsl_ign_plugins::systems::LightControlSystem::ISystemConfigure,
    stsl_ign_plugins::systems::LightControlSystem::ISystemPreUpdate
)
