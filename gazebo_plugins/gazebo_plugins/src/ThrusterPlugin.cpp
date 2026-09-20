#include <ThrusterPlugin.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/plugin/Register.hh>

#include <PropellerDynamicsFactory.h>
#include <ThrusterConverterFactory.h>
#include <WorldWrench.h>

namespace {

// Parses a whitespace separated list of doubles.
std::vector<double> ParseDoubles(const std::string &_text) {
  std::vector<double> values;
  std::istringstream stream(_text);
  double value = 0.0;
  while (stream >> value)
    values.push_back(value);
  return values;
}

// Reads a 3 element vector tag, returning `_defaultValue` when the tag is
// absent or malformed.
Eigen::Vector3d ParseVector3(const std::shared_ptr<const sdf::Element> &_sdf,
                             const std::string &_tag,
                             const Eigen::Vector3d &_defaultValue) {
  if (!_sdf->HasElement(_tag))
    return _defaultValue;
  const std::vector<double> values = ParseDoubles(_sdf->Get<std::string>(_tag));
  if (values.size() < 3)
    return _defaultValue;
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

// Legacy topic layout: /<model>/thrusters/id_<id>/
std::string BuildTopicPrefix(const std::string &_modelName, int _thrusterId) {
  std::ostringstream stream;
  stream << "/" << _modelName << "/thrusters/id_" << _thrusterId << "/";
  return stream.str();
}

gz::math::Vector3d ToGz(const Eigen::Vector3d &_vector) {
  return gz::math::Vector3d(_vector.x(), _vector.y(), _vector.z());
}

// Reads an optional scalar tag, keeping the current value when absent.
double ReadDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                  const std::string &_tag, double _current) {
  if (!_sdf->HasElement(_tag))
    return _current;
  return _sdf->Get<double>(_tag);
}

// Efficiency factors are ratios and must stay within [0, 1].
double ReadEfficiency(const std::shared_ptr<const sdf::Element> &_sdf,
                      const std::string &_tag) {
  const double value = ReadDouble(_sdf, _tag, 1.0);
  if (value < 0.0 || value > 1.0) {
    gzwarn << "[ThrusterPlugin] invalid " << _tag
            << " (must be within [0, 1]), using 1.0" << std::endl;
    return 1.0;
  }
  return value;
}

}  // namespace

ThrusterPlugin::ThrusterPlugin() = default;

ThrusterPlugin::~ThrusterPlugin() = default;

void ThrusterPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) {
  this->m_model = gz::sim::Model(_entity);

  std::string linkName = "base_link";
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");

  const auto linkEntity = this->m_model.LinkByName(_ecm, linkName);
  if (linkEntity == gz::sim::kNullEntity) {
    gzwarn << "[ThrusterPlugin] link '" << linkName << "' not found in model '"
            << this->m_model.Name(_ecm) << "'" << std::endl;
    return;
  }
  this->m_link = gz::sim::Link(linkEntity);

  // Gazebo creates components on demand: without these checks the physics
  // system does not report the link state and Link::WorldPose() keeps
  // returning nullopt.
  this->m_link.EnableVelocityChecks(_ecm);
  this->m_link.EnableAccelerationChecks(_ecm);

  // ---- Propeller dynamics and thrust conversion (selected by SDF type) ----
  if (!_sdf->HasElement("dynamics")) {
    gzwarn << "[ThrusterPlugin] '<dynamics>' element is missing" << std::endl;
    return;
  }
  this->m_propellerDynamics =
      PropellerDynamicsFactory::GetInstance().Create(
          *_sdf->FindElement("dynamics"));
  if (!this->m_propellerDynamics) {
    gzwarn << "[ThrusterPlugin] could not create the propeller dynamics"
            << std::endl;
    return;
  }

  if (!_sdf->HasElement("conversion")) {
    gzwarn << "[ThrusterPlugin] '<conversion>' element is missing" << std::endl;
    return;
  }
  this->m_converter = ThrusterConverterFactory::GetInstance().Create(
      *_sdf->FindElement("conversion"));
  if (!this->m_converter) {
    gzwarn << "[ThrusterPlugin] could not create the thrust converter"
            << std::endl;
    return;
  }

  // ---- Geometry of the thruster in the link frame -------------------------
  Eigen::Vector3d axis =
      ParseVector3(_sdf, "thruster_axis", Eigen::Vector3d::UnitX());
  if (axis.norm() > 0.0) {
    this->m_axis = axis.normalized();
  } else {
    gzwarn << "[ThrusterPlugin] 'thruster_axis' is a zero vector, using the "
               "link x axis" << std::endl;
  }
  this->m_applicationPoint =
      ParseVector3(_sdf, "application_point", Eigen::Vector3d::Zero());
  this->m_centerOfMass =
      ParseVector3(_sdf, "center_of_mass", Eigen::Vector3d::Zero());

  // ---- Command and thrust limits -----------------------------------------
  this->m_clampMin = ReadDouble(_sdf, "clamp_min", this->m_clampMin);
  this->m_clampMax = ReadDouble(_sdf, "clamp_max", this->m_clampMax);
  if (this->m_clampMin >= this->m_clampMax) {
    gzwarn << "[ThrusterPlugin] 'clamp_max' must be greater than 'clamp_min', "
               "using the defaults" << std::endl;
    this->m_clampMin = std::numeric_limits<double>::lowest();
    this->m_clampMax = std::numeric_limits<double>::max();
  }

  this->m_thrustMin = ReadDouble(_sdf, "thrust_min", this->m_thrustMin);
  this->m_thrustMax = ReadDouble(_sdf, "thrust_max", this->m_thrustMax);
  if (this->m_thrustMin >= this->m_thrustMax) {
    gzwarn << "[ThrusterPlugin] 'thrust_max' must be greater than "
               "'thrust_min', using the defaults" << std::endl;
    this->m_thrustMin = std::numeric_limits<double>::lowest();
    this->m_thrustMax = std::numeric_limits<double>::max();
  }

  this->m_gain = ReadDouble(_sdf, "gain", this->m_gain);
  this->m_thrustEfficiency = ReadEfficiency(_sdf, "thrust_efficiency");
  this->m_propellerEfficiency = ReadEfficiency(_sdf, "propeller_efficiency");

  // ---- Optional rotor joint (visualization only) --------------------------
  if (_sdf->HasElement("joint_name")) {
    const std::string jointName = _sdf->Get<std::string>("joint_name");
    this->m_jointEntity = this->m_model.JointByName(_ecm, jointName);
    if (this->m_jointEntity == gz::sim::kNullEntity)
      gzwarn << "[ThrusterPlugin] joint '" << jointName
              << "' not found, the rotor will not be animated" << std::endl;
  }

  if (_sdf->HasElement("debug"))
    this->m_debugMode = _sdf->Get<bool>("debug");

  // ---- Input / output topics ---------------------------------------------
  int thrusterId = -1;
  if (_sdf->HasElement("thruster_id"))
    thrusterId = _sdf->Get<int>("thruster_id");
  const std::string modelName = this->m_model.Name(_ecm);
  const std::string topicPrefix = BuildTopicPrefix(modelName, thrusterId);

  this->m_node = std::make_shared<gz::transport::Node>();

  this->m_node->Subscribe(topicPrefix + "input", &ThrusterPlugin::OnCommand,
                          this);
  this->m_thrustPublisher =
      this->m_node->Advertise<gz::msgs::Vector3d>(topicPrefix + "thrust");

  gzmsg << "[ThrusterPlugin] attached to link '" << linkName << "' of model '"
         << modelName << "', axis ("
         << this->m_axis.transpose() << "), application point ("
         << this->m_applicationPoint.transpose() << "), centre of gravity ("
         << this->m_centerOfMass.transpose() << "), input topic '"
         << topicPrefix << "input'" << std::endl;
}

void ThrusterPlugin::OnCommand(const gz::msgs::Double &_msg) {
  this->m_command.store(_msg.data());
}

ThrusterPlugin::ThrusterState ThrusterPlugin::Evaluate(double _timeSeconds) {
  // Command limits, then the optional gain, then the propeller dynamics.
  const double command = std::max(
      this->m_clampMin, std::min(this->m_command.load(), this->m_clampMax));
  double propellerSpeed =
      this->m_propellerEfficiency *
      this->m_propellerDynamics->Update(this->m_gain * command, _timeSeconds);
  if (std::isnan(propellerSpeed))
    propellerSpeed = 0.0;

  // Thrust conversion, then the thrust limits.
  double thrust = this->m_thrustEfficiency *
                  this->m_converter->Convert(propellerSpeed);
  if (std::isnan(thrust))
    thrust = 0.0;

  ThrusterState state;
  state.propellerSpeed = propellerSpeed;
  state.thrust = std::max(this->m_thrustMin, std::min(thrust, this->m_thrustMax));
  return state;
}

void ThrusterPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm) {
  if (this->m_link.Entity() == gz::sim::kNullEntity ||
      !this->m_propellerDynamics || !this->m_converter)
    return;

  const double timeSeconds =
      std::chrono::duration<double>(_info.simTime).count();
  const ThrusterState state = this->Evaluate(timeSeconds);
  const double thrust = state.thrust;

  const auto pose = this->m_link.WorldPose(_ecm);
  if (!pose)
    return;

  // ---- Wrench in the link frame ------------------------------------------
  const Eigen::Vector3d forceBody = thrust * this->m_axis;
  const Eigen::Vector3d armBody = this->m_applicationPoint - this->m_centerOfMass;
  // Moment of the thrust about the centre of gravity of the vehicle.
  const Eigen::Vector3d torqueBody = armBody.cross(forceBody);

  // ---- Link -> world ------------------------------------------------------
  const auto &rotation = pose->Rot();
  const Eigen::Quaterniond orientation(rotation.W(), rotation.X(),
                                       rotation.Y(), rotation.Z());
  const Eigen::Vector3d forceWorld = orientation * forceBody;
  const Eigen::Vector3d torqueWorld = orientation * torqueBody;

  // Explicit accumulation: several thrusters (and the buoyancy plugin) write
  // to the same link in the same step.
  AddWorldWrench(_ecm, this->m_link.Entity(), ToGz(forceWorld),
                 ToGz(torqueWorld));

  // Optionally spin the rotor joint so that the propeller follows the
  // dynamics (visualization only, no reaction on the body).
  if (this->m_jointEntity != gz::sim::kNullEntity) {
    namespace components = gz::sim::components;
    if (!_ecm.Component<components::JointVelocityCmd>(this->m_jointEntity)) {
      _ecm.CreateComponent(
          this->m_jointEntity,
          components::JointVelocityCmd({state.propellerSpeed}));
    } else {
      _ecm.SetComponentData<components::JointVelocityCmd>(
          this->m_jointEntity, {state.propellerSpeed});
    }
  }

  // Publish the world frame thrust force for monitoring.
  if (this->m_thrustPublisher.HasConnections()) {
    gz::msgs::Vector3d message;
    message.set_x(forceWorld.x());
    message.set_y(forceWorld.y());
    message.set_z(forceWorld.z());
    this->m_thrustPublisher.Publish(message);
  }

  if (this->m_debugMode && ++this->m_debugCounter % 100 == 0)
    gzmsg << "[ThrusterPlugin][debug] t=" << _info.simTime.count()
           << " ns, command=" << this->m_command.load() << ", thrust=" << thrust
           << ", F_body=(" << forceBody.transpose() << "), M_body=("
           << torqueBody.transpose() << ")" << std::endl;
}

GZ_ADD_PLUGIN(ThrusterPlugin, gz::sim::System,
                    ThrusterPlugin::ISystemConfigure,
                    ThrusterPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ThrusterPlugin, "thruster")
