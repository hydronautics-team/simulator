#pragma once

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/wrench.pb.h>

// Adds a world frame wrench to a link.
//
// Gazebo keeps a single external wrench per link, and several systems may
// write to it during the same step (two thrusters on one body, a thruster
// plus the buoyancy plugin). Accumulating the component explicitly keeps all
// contributions instead of letting the writers depend on their order.
inline void AddWorldWrench(ignition::gazebo::EntityComponentManager &_ecm,
                           const ignition::gazebo::Entity &_entity,
                           const ignition::math::Vector3d &_force,
                           const ignition::math::Vector3d &_torque) {
  using WrenchCmd = ignition::gazebo::components::ExternalWorldWrenchCmd;

  ignition::msgs::Wrench wrench;
  if (auto *component = _ecm.Component<WrenchCmd>(_entity)) {
    wrench = component->Data();
    ignition::msgs::Set(wrench.mutable_force(),
                        ignition::msgs::Convert(wrench.force()) + _force);
    ignition::msgs::Set(wrench.mutable_torque(),
                        ignition::msgs::Convert(wrench.torque()) + _torque);
    _ecm.SetComponentData<WrenchCmd>(_entity, wrench);
  } else {
    ignition::msgs::Set(wrench.mutable_force(), _force);
    ignition::msgs::Set(wrench.mutable_torque(), _torque);
    _ecm.CreateComponent(_entity, WrenchCmd(wrench));
  }
}
