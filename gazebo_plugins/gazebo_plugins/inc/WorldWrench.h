#pragma once

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>

// Adds a world frame wrench to a link.
//
// Gazebo keeps a single external wrench per link, and several systems may
// write to it during the same step (two thrusters on one body, a thruster
// plus the buoyancy plugin). Accumulating the component explicitly keeps all
// contributions instead of letting the writers depend on their order.
inline void AddWorldWrench(gz::sim::EntityComponentManager &_ecm,
                           const gz::sim::Entity &_entity,
                           const gz::math::Vector3d &_force,
                           const gz::math::Vector3d &_torque) {
  using WrenchCmd = gz::sim::components::ExternalWorldWrenchCmd;

  gz::msgs::Wrench wrench;
  if (auto *component = _ecm.Component<WrenchCmd>(_entity)) {
    wrench = component->Data();
    gz::msgs::Set(wrench.mutable_force(),
                  gz::msgs::Convert(wrench.force()) + _force);
    gz::msgs::Set(wrench.mutable_torque(),
                  gz::msgs::Convert(wrench.torque()) + _torque);
    _ecm.SetComponentData<WrenchCmd>(_entity, wrench);
  } else {
    gz::msgs::Set(wrench.mutable_force(), _force);
    gz::msgs::Set(wrench.mutable_torque(), _torque);
    _ecm.CreateComponent(_entity, WrenchCmd(wrench));
  }
}
