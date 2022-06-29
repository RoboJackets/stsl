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

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>

namespace stsl_gazebo_plugins
{

class HideLightVisualsPlugin : public gazebo::VisualPlugin
{
public:
  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    scene_ = visual->GetScene();

    LoadLightNames(sdf);

    if (light_names_.empty()) {
      gzwarn <<
        "HideLightVisualsPlugin loaded with no light names specified. Are you missing <light> "
        "elements in the plugin SDF?" << std::endl;
      return;
    }

    gzmsg << light_names_.size() << " light names loaded." << std::endl;


    pre_render_connection_ =
      gazebo::event::Events::ConnectPreRender(
      std::bind(
        &HideLightVisualsPlugin::PreRender,
        this));
  }

private:
  gazebo::rendering::ScenePtr scene_;
  std::vector<std::string> light_names_;
  gazebo::event::ConnectionPtr pre_render_connection_;


  void LoadLightNames(sdf::ElementPtr sdf)
  {
    for (auto element = sdf->GetElement("light");
      element != nullptr;
      element = element->GetNextElement("light") )
    {
      light_names_.push_back(element->GetValue()->GetAsString());
    }
  }

  void PreRender()
  {
    const auto light_count = scene_->LightCount();
    gzmsg << "There are " << light_count << " lights." << std::endl;
    for (auto i = 0; i < light_count; ++i) {
      auto light = scene_->LightByIndex(i);
      if (light) {
        gzmsg << "Light: " << light->Name() << std::endl;
      }
    }

    for (const auto light_name : light_names_) {
      auto light = scene_->LightByName(light_name);
      if (light) {
        light->ShowVisual(false);
      } else {
        gzwarn << "Could not find light with name: " << light_name << std::endl;
      }
    }
    // Release the event connection, since we only need to run this procedure once.
    pre_render_connection_.reset();
  }
};

GZ_REGISTER_VISUAL_PLUGIN(HideLightVisualsPlugin)

}  // namespace stsl_gazebo_plugins
