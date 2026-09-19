#include <UnderwaterObjectPlugin.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>
#include <vector>

#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>
namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Parses a whitespace separated list of doubles.
std::vector<double> ParseDoubles(const std::string &_text) {
  std::vector<double> values;
  std::istringstream stream(_text);
  double value = 0.0;
  while (stream >> value)
    values.push_back(value);
  return values;
}

// Constant diagonal of `_size` entries.
void ParseDiagonal(const std::shared_ptr<const sdf::Element> &_sdf,
                   const std::string &_tag, Eigen::Matrix<double, 6, 1> &_out) {
  _out = Eigen::Matrix<double, 6, 1>::Zero();
  if (!_sdf->HasElement(_tag))
    return;
  const std::vector<double> values = ParseDoubles(_sdf->Get<std::string>(_tag));
  if (values.size() >= 6) {
    for (std::size_t i = 0; i < 6; ++i)
      _out(i) = values[i];
  } else if (!values.empty()) {
    _out.setConstant(values[0]);
  }
}

// 6x6 matrix from 36 values (row-major) or from a 6 element diagonal.
Matrix6d ParseMatrix(const std::shared_ptr<const sdf::Element> &_sdf,
                     const std::string &_tag) {
  Matrix6d matrix = Matrix6d::Zero();
  if (!_sdf->HasElement(_tag))
    return matrix;
  const std::vector<double> values = ParseDoubles(_sdf->Get<std::string>(_tag));
  if (values.size() >= 36) {
    for (std::size_t i = 0; i < 36; ++i)
      matrix(i / 6, i % 6) = values[i];
  } else if (values.size() >= 6) {
    for (std::size_t i = 0; i < 6; ++i)
      matrix(i, i) = values[i];
  } else if (!values.empty()) {
    matrix.diagonal().setConstant(values[0]);
  }
  return matrix;
}

Eigen::Vector3d ParseVector3(const std::shared_ptr<const sdf::Element> &_sdf,
                             const std::string &_tag) {
  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
  if (!_sdf->HasElement(_tag))
    return vector;
  const std::vector<double> values = ParseDoubles(_sdf->Get<std::string>(_tag));
  if (values.size() >= 3)
    vector << values[0], values[1], values[2];
  return vector;
}

}  // namespace

UnderwaterObjectPlugin::UnderwaterObjectPlugin() = default;

UnderwaterObjectPlugin::~UnderwaterObjectPlugin() = default;

void UnderwaterObjectPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &) {
  this->m_model = ignition::gazebo::Model(_entity);

  std::string linkName = "base_link";
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");

  const auto linkEntity = this->m_model.LinkByName(_ecm, linkName);
  if (linkEntity == ignition::gazebo::kNullEntity) {
    ignwarn << "[UnderwaterObjectPlugin] link '" << linkName
            << "' not found in model '" << this->m_model.Name(_ecm) << "'"
            << std::endl;
    return;
  }
  this->m_baseLink = ignition::gazebo::Link(linkEntity);

  if (_sdf->HasElement("debug"))
    this->m_debugMode = _sdf->Get<bool>("debug");

  // ---- Buoyancy model parameters -----------------------------------------
  double volume = 0.0;
  double fluidDensity = 1028.0;
  Eigen::Vector3d buoyancyCenter = Eigen::Vector3d::Zero();
  Eigen::Vector3d massCenter = Eigen::Vector3d::Zero();
  if (_sdf->HasElement("volume"))
    volume = _sdf->Get<double>("volume");
  if (_sdf->HasElement("fluid_density"))
    fluidDensity = _sdf->Get<double>("fluid_density");
  buoyancyCenter = ParseVector3(_sdf, "center_of_buoyancy");
  massCenter = ParseVector3(_sdf, "center_of_mass");

  // World gravity
  Eigen::Vector3d gravity(0.0, 0.0, -9.8);
  {
    const auto world = _ecm.EntityByComponents(
        ignition::gazebo::components::World());
    const auto gravityComp =
        _ecm.Component<ignition::gazebo::components::Gravity>(world);
    if (gravityComp)
      gravity = Eigen::Vector3d(gravityComp->Data().X(),
                                gravityComp->Data().Y(),
                                gravityComp->Data().Z());
  }

  // ---- Hydrodynamic model parameters --------------------------------------
  // Mass matrix M = M_RB + M_A (the rigid-body part is optional in the SDF;
  // by default only the added mass is configured, since the physics engine
  // already accounts for the link inertia).
  Matrix6d massMatrix = Matrix6d::Zero();
  double mass = 0.0;
  if (_sdf->HasElement("mass"))
    mass = _sdf->Get<double>("mass");
  Eigen::Vector3d inertia = Eigen::Vector3d::Zero();
  if (_sdf->HasElement("inertia")) {
    const auto values = ParseDoubles(_sdf->Get<std::string>("inertia"));
    if (values.size() >= 3)
      inertia << values[0], values[1], values[2];
  }
  massMatrix.diagonal() << mass, mass, mass, inertia(0), inertia(1), inertia(2);
  massMatrix += ParseMatrix(_sdf, "added_mass");

  Vector6d linearDamping = Vector6d::Zero();
  Vector6d linearDampingForward = Vector6d::Zero();
  Vector6d quadraticDamping = Vector6d::Zero();
  ParseDiagonal(_sdf, "linear_damping", linearDamping);
  ParseDiagonal(_sdf, "linear_damping_forward_speed", linearDampingForward);
  ParseDiagonal(_sdf, "quadratic_damping", quadraticDamping);

  // ---- Create the models (once, immutable afterwards) ---------------------
  this->m_buoyancyModel = std::make_unique<BuoyancyModel>(
      volume, fluidDensity, Eigen::Vector3d(buoyancyCenter),
      Eigen::Vector3d(massCenter), Eigen::Vector3d(gravity));
  this->m_hydroModel = std::make_unique<HydrodynamicModel>(
      massMatrix, linearDamping, linearDampingForward, quadraticDamping);

  ignmsg << "[UnderwaterObjectPlugin] attached to link '" << linkName
         << "' of model '" << this->m_model.Name(_ecm) << "', volume "
         << volume << " m^3, rho " << fluidDensity
         << " kg/m^3, debug=" << (this->m_debugMode ? "on" : "off")
         << std::endl;
}

UnderwaterObjectPlugin::Vector6d UnderwaterObjectPlugin::ComputeAcceleration(
    const Vector6d &_velocity, double _dtSeconds) {
  if (!this->m_hasLastVelocity || _dtSeconds <= 0.0) {
    this->m_hasLastVelocity = true;
    this->m_lastVelocity = _velocity;
    return this->m_filteredAcceleration;
  }

  for (int i = 0; i < 6; ++i) {
    const double raw =
        std::max(-kAccelerationClamp,
                 std::min(kAccelerationClamp,
                          (_velocity(i) - this->m_lastVelocity(i)) /
                              _dtSeconds));
    this->m_filteredAcceleration(i) =
        (1.0 - kAccelerationFilterAlpha) * this->m_filteredAcceleration(i) +
        kAccelerationFilterAlpha * raw;
  }
  this->m_lastVelocity = _velocity;
  return this->m_filteredAcceleration;
}

void UnderwaterObjectPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) {
  if (this->m_baseLink.Entity() == ignition::gazebo::kNullEntity ||
      !this->m_buoyancyModel || !this->m_hydroModel)
    return;

  // Checks must be enabled so that the physics engine publishes the pose /
  // velocity components and applies externally commanded forces.
  this->m_baseLink.EnableVelocityChecks(_ecm, true);
  this->m_baseLink.EnableAccelerationChecks(_ecm, true);

  const auto pose = this->m_baseLink.WorldPose(_ecm);
  if (!pose)
    return;  // physics has not started yet

  const auto linearVelocity = this->m_baseLink.WorldLinearVelocity(_ecm);
  const auto angularVelocity = this->m_baseLink.WorldAngularVelocity(_ecm);
  if (!linearVelocity || !angularVelocity)
    return;

  // World -> body (rotation as an Eigen quaternion: w, x, y, z)
  const auto &q = pose->Rot();
  const Eigen::Quaterniond rotation(q.W(), q.X(), q.Y(), q.Z());
  const Eigen::Vector3d bodyLinear = rotation.conjugate() * Eigen::Vector3d(
      linearVelocity->X(), linearVelocity->Y(), linearVelocity->Z());
  const Eigen::Vector3d bodyAngular = rotation.conjugate() * Eigen::Vector3d(
      angularVelocity->X(), angularVelocity->Y(), angularVelocity->Z());

  Vector6d velocity;
  velocity << bodyLinear, bodyAngular;

  const double timeSeconds = std::chrono::duration<double>(_info.simTime).count();
  const Vector6d acceleration =
      this->ComputeAcceleration(velocity, timeSeconds - this->m_lastTimeSeconds);
  this->m_lastTimeSeconds = timeSeconds;

  // ---- Model wrenches (body frame) ----------------------------------------
  const Vector6d buoyancy = this->m_buoyancyModel->ComputeRestoringVector();
  const Vector6d hydrodynamic =
      this->m_hydroModel->ComputeHydrodynamicForces(velocity, acceleration);

  const Eigen::Vector3d forceBody =
      buoyancy.head<3>() + hydrodynamic.head<3>();
  const Eigen::Vector3d torqueBody =
      buoyancy.tail<3>() + hydrodynamic.tail<3>();

  // Body -> world
  const Eigen::Vector3d worldForce = rotation * forceBody;
  const Eigen::Vector3d worldTorque = rotation * torqueBody;
  this->m_baseLink.AddWorldWrench(_ecm,
      ignition::math::Vector3d(worldForce.x(), worldForce.y(),
                               worldForce.z()),
      ignition::math::Vector3d(worldTorque.x(), worldTorque.y(),
                               worldTorque.z()));

  if (this->m_debugMode && ++this->m_debugCounter % 100 == 0)
    ignmsg << "[UnderwaterObjectPlugin][debug] t=" << _info.simTime.count()
           << " ns, v_body=(" << bodyLinear.transpose() << "), F_body=("
           << forceBody.transpose() << "), M_body=(" << torqueBody.transpose()
           << ")" << std::endl;
}

IGNITION_ADD_PLUGIN(UnderwaterObjectPlugin, ignition::gazebo::System,
                    UnderwaterObjectPlugin::ISystemConfigure,
                    UnderwaterObjectPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(UnderwaterObjectPlugin, "underwater_object")
