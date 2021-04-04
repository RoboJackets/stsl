#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace stsl_gazebo_plugins
{

gazebo::physics::LinkPtr findLinkForLight(
    const gazebo::physics::ModelPtr& model,
    const std::string& light_name, 
    const std::string& link_name)
{
    auto child_link = model->GetChildLink(link_name);
    if(child_link && child_link->GetSDF()->HasElement("light"))
    {
        auto sdf_light = child_link->GetSDF()->GetElement("light");
        while(sdf_light)
        {
            if(sdf_light->Get<std::string>("name") == light_name)
            {
                return child_link;
            }
            sdf_light = sdf_light->GetNextElement("light");
        }
    }
    for(auto nested_model : model->NestedModels())
    {
        auto found_link = findLinkForLight(nested_model, light_name, link_name);
        if(found_link)
            return found_link;
    }
    return nullptr;
}

double getLightRange(const gazebo::physics::LinkPtr& link, const std::string& light_name)
{
    if(!link->GetSDF()->HasElement("light"))
    {
        gzerr << "Link has no <light> element!" << std::endl;
        return 0.0;
    }
    auto sdf_light = link->GetSDF()->GetElement("light");
    while(sdf_light)
    {
        if(sdf_light->Get<std::string>("name") == light_name)
        {
            return sdf_light->GetElement("attenuation")->Get<double>("range");
        }
        sdf_light = sdf_light->GetNextElement("light");
    }
    gzerr << "Link has no <light> element with name " << light_name << std::endl;
    return 0.0;
}

gazebo::msgs::Visual getVisualProperties(const gazebo::physics::LinkPtr& link, const std::string& name)
{
    gazebo::msgs::Link link_msg;
    link->FillMsg(link_msg);
    const auto& visuals = link_msg.visual();
    const auto expected_visual_name = link->GetScopedName() + "::" + name;
    auto found_iter = std::find_if(visuals.begin(), visuals.end(), [&expected_visual_name](const auto& visual){
        return visual.name() == expected_visual_name;
    });
    if(found_iter == visuals.end())
    {
        gzerr << "Could not find visual named " << name << std::endl;
        return {};
    }
    return *found_iter;
}

class RosLightControlPlugin : public gazebo::ModelPlugin
{
public:

    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override
    {
        if(!sdf->HasElement("light_id"))
        {
            gzerr << "Parameter <light_id> is missing." << std::endl;
        }

        const auto light_id = sdf->Get<std::string>("light_id");

        const int sep_pos = light_id.rfind("/");
        const auto light_name = light_id.substr(sep_pos+1);
        const auto link_name = light_id.substr(0, sep_pos);

        const auto link = findLinkForLight(parent, light_name, link_name);

        const auto visual_props = getVisualProperties(link, light_name);

        light_on_range_ = getLightRange(link, light_name);
        light_off_transparency_ = visual_props.transparency();
        light_on_emissive_color_ = gazebo::msgs::Convert(visual_props.material().emissive());

        gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
        gazebo_node_->Init();

        light_publisher_ = gazebo_node_->Advertise<gazebo::msgs::Light>("~/light/modify");
        light_publisher_->WaitForConnection();

        light_msg_.set_name(link->GetScopedName() + "::" + light_name);

        visual_publisher_ = gazebo_node_->Advertise<gazebo::msgs::Visual>("~/visual");

        visual_msg_.set_name(link->GetScopedName() + "::" + light_name);
        visual_msg_.set_parent_name(link->GetScopedName());
        uint32_t visual_id;
        link->VisualId(light_name, visual_id);
        visual_msg_.set_id(visual_id);

        ros_node_ = gazebo_ros::Node::Get(sdf);

        light_command_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>("/light_control/" + light_name, 1, std::bind(&RosLightControlPlugin::onCommandMsg, this, std::placeholders::_1));

        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RosLightControlPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if(!commanded_state_changed_)
            return;

        if(commanded_light_state_)
            turnLightOn();
        else
            turnLightOff();

        commanded_state_changed_ = false;
    }

private:
    gazebo::event::ConnectionPtr update_connection_;

    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::PublisherPtr light_publisher_;
    gazebo::transport::PublisherPtr visual_publisher_;

    gazebo::msgs::Light light_msg_;
    gazebo::msgs::Visual visual_msg_;

    gazebo_ros::Node::SharedPtr ros_node_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr light_command_sub_;

    std::mutex state_mutex_;
    bool commanded_light_state_{false};
    bool commanded_state_changed_{true};

    double light_on_range_{0.0};
    ignition::math::Color light_on_emissive_color_;
    double light_off_transparency_{0.2};

    void onCommandMsg(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        commanded_state_changed_ = commanded_light_state_ != msg->data;
        commanded_light_state_ = msg->data;
    }

    void turnLightOn()
    {
        light_msg_.set_range(light_on_range_);
        light_publisher_->Publish(light_msg_);

        visual_msg_.set_transparency(0.0);
        gazebo::msgs::Set(visual_msg_.mutable_material()->mutable_emissive(), light_on_emissive_color_);
        visual_publisher_->Publish(visual_msg_);
    }

    void turnLightOff()
    {
        light_msg_.set_range(0.0);
        light_publisher_->Publish(light_msg_);

        visual_msg_.set_transparency(light_off_transparency_);
        gazebo::msgs::Set(visual_msg_.mutable_material()->mutable_emissive(), ignition::math::Color(0,0,0));
        visual_publisher_->Publish(visual_msg_);
    }
};

GZ_REGISTER_MODEL_PLUGIN(RosLightControlPlugin)
    
} // namespace stsl_gazebo_plugins
