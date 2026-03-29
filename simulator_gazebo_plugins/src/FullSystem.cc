/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Remove unused includes if your plugin doesn't need them.
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "simulator_gazebo_plugins/FullSystem.hh"

//#include <gz/sim/Model.hh>



// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    simulator_gazebo_plugins::FullSystem,
    gz::sim::System,
    simulator_gazebo_plugins::FullSystem::ISystemConfigure,
    simulator_gazebo_plugins::FullSystem::ISystemPreUpdate,
    simulator_gazebo_plugins::FullSystem::ISystemUpdate,
    simulator_gazebo_plugins::FullSystem::ISystemPostUpdate
)

namespace simulator_gazebo_plugins 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{

  (void)_element;
  (void)_ecm;
  (void)_eventManager;

  // ##############################################################################################################
  // Store the pointer to the model
  //this->model = _model;

  //std::cout<< "Model Name=" << this->model->GetName() << std::endl;
  this->model = gz::sim::Model(_entity);

  // Read property from SDF
  //auto linkName = _element->Get<std::string>("link_name");
  // Create model object to access convenient functions
  //auto model = gz::sim::Model(_entity);
  // Get link entity
  //this->linkEntity = model.LinkByName(_ecm, linkName);

  //double vel = 0.4;

  //std::cout << "model_vel= " << vel << std::endl;

  //model->SetLinearVel(ignition::math::Vector3d(0, 0, vel));

  //math::Pose3d pose;
  //this->model.SetWorldPoseCmd(_ecm, math::Pose3d(7, 2, 3, 0, 0, 0, 0));
  //this->model.SetWorldPoseCmd(_ecm, pose);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  //this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyModelPlugin::OnUpdate, this));
  // ##############################################################################################################

  igndbg << "simulator_gazebo_plugins::FullSystem::Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  (void)_ecm;
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "simulator_gazebo_plugins::FullSystem::PreUpdate" << std::endl;
  }
}

void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  (void)_ecm;
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "simulator_gazebo_plugins::FullSystem::Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm)
{
  (void)_ecm;
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "simulator_gazebo_plugins::FullSystem::PostUpdate" << std::endl;
  }
}



}  // namespace simulator_gazebo_plugins
