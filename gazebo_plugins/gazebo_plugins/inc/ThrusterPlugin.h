#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <string>

#include <eigen3/Eigen/Core>

#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>

#include <IPropellerDynamics.h>
#include <IThrusterConverter.h>

// Facade for one thruster: the incoming rotor speed command goes through the
// propeller dynamics and then through the thrust conversion, and the resulting
// wrench is applied to the body link as
//
//   force  = thrust * thruster_axis                     (in the link frame)
//   torque = (application_point - center_of_mass) x force
//
// The legacy classic plugin only pushed the body along the thruster axis; the
// moment of that force about the centre of gravity of the vehicle is added
// here explicitly.
//
// <link_name> is the body link that receives the wrench, and the arm is given
// explicitly by <application_point> / <center_of_mass>, so the moment is never
// counted twice (do not point <link_name> at a separate thruster link).
//
// SDF (all lengths are expressed in the frame of <link_name>):
//
//   <plugin filename="libthruster.so" name="thruster">
//     <link_name>base_link</link_name>
//     <thruster_id>0</thruster_id>
//     <thruster_axis>1 0 0</thruster_axis>
//     <application_point>0.3 -0.2 0</application_point>
//     <center_of_mass>0 0 0.05</center_of_mass>
//     <joint_name>propeller_joint</joint_name>
//     <dynamics>
//       <type>ZeroOrder</type>
//     </dynamics>
//     <conversion>
//       <type>Basic</type>
//       <rotorConstant>0.00081</rotorConstant>
//     </conversion>
//     <clamp_min>-100</clamp_min>
//     <clamp_max>100</clamp_max>
//     <thrust_min>-50</thrust_min>
//     <thrust_max>50</thrust_max>
//     <gain>1.0</gain>
//     <thrust_efficiency>1.0</thrust_efficiency>
//     <propeller_efficiency>1.0</propeller_efficiency>
//     <debug>0</debug>
//   </plugin>
//
// The rotor speed command is received on the legacy topic
// "/<model>/thrusters/id_<thruster_id>/input" (gz.msgs.Double, field
// `data`); the resulting thrust is published as a world frame force on
// "/<model>/thrusters/id_<thruster_id>/thrust" (gz.msgs.Vector3d).
class ThrusterPlugin : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate {
public:
  ThrusterPlugin();
  ~ThrusterPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  // Callback of the input topic: stores the last rotor speed command.
  void OnCommand(const gz::msgs::Double &_msg);

  // Runs the command through clamp -> gain -> dynamics -> conversion ->
  // thrust limits.
  struct ThrusterState {
    double propellerSpeed{0.0};
    double thrust{0.0};
  };
  ThrusterState Evaluate(double _timeSeconds);

private:
  gz::sim::Model m_model;
  gz::sim::Link m_link;
  gz::sim::Entity m_jointEntity{gz::sim::kNullEntity};

  // Models created once in Configure (via the SDF factories).
  std::unique_ptr<IPropellerDynamics> m_propellerDynamics;
  std::unique_ptr<IThrusterConverter> m_converter;

  // Configuration, set once in Configure and only read afterwards.
  Eigen::Vector3d m_axis{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d m_applicationPoint{Eigen::Vector3d::Zero()};
  Eigen::Vector3d m_centerOfMass{Eigen::Vector3d::Zero()};
  double m_clampMin{std::numeric_limits<double>::lowest()};
  double m_clampMax{std::numeric_limits<double>::max()};
  double m_thrustMin{std::numeric_limits<double>::lowest()};
  double m_thrustMax{std::numeric_limits<double>::max()};
  double m_gain{1.0};
  double m_thrustEfficiency{1.0};
  double m_propellerEfficiency{1.0};

  // Runtime state.
  std::atomic<double> m_command{0.0};
  std::shared_ptr<gz::transport::Node> m_node;
  gz::transport::Node::Publisher m_thrustPublisher;
  bool m_debugMode{false};
  unsigned int m_debugCounter{0};
};
