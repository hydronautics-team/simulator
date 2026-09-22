#pragma once

#include <memory>
#include <string>

#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <WaterPressureModel.h>

// Water pressure (depth) sensor of a link: publishes the hydrostatic pressure
// of the surrounding water as gz::msgs::FluidPressure, so that a real depth
// sensor (pressure sensor) can be emulated.
//
// SDF:
//
//   <plugin filename="libwater_pressure.so" name="water_pressure">
//     <link_name>base_link</link_name>
//     <surface_z>0.0</surface_z>
//     <fluid_density>1028.0</fluid_density>
//     <update_rate>10</update_rate>
//     <noise_stddev>0.0</noise_stddev>
//     <topic>/my_robot/sensors/pressure</topic>   <!-- optional -->
//     <debug>0</debug>
//   </plugin>
//
// The message is published on "/<model>/sensors/pressure" by default. Depth
// (m) can be recovered from the pressure as
//   depth = (pressure - 101325) / (fluid_density * 9.80665).
class WaterPressurePlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate {
public:
  WaterPressurePlugin();
  ~WaterPressurePlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::sim::Model m_model;
  gz::sim::Link m_link;
  std::string m_linkName;

  // Model created once in Configure (never mutated afterwards except for the
  // noise generator).
  std::unique_ptr<WaterPressureModel> m_pressureModel;

  std::shared_ptr<gz::transport::Node> m_node;
  gz::transport::Node::Publisher m_publisher;

  // Publishing throttle: 1 / update_rate, in seconds.
  double m_updatePeriodSeconds{0.1};
  double m_lastPublishSeconds{0.0};
  bool m_hasPublished{false};

  bool m_debugMode{false};
  unsigned int m_debugCounter{0};
};
