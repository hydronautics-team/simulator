// Copyright (c) 2026 simulator authors.
//
// Gazebo Sim (ros_gz_sim stack) System plugin for an underwater body:
//   * buoyancy (Archimedes) with the body split into flat slices so that the
//     submerged volume / force drop as slices cross the water surface (z=0);
//   * hydrodynamic forces following the Fossen model, as in
//     uuv_gazebo_plugins/HydrodynamicModel.cpp:
//       tau = -( D(nu)*nu  +  Ma*nu_dot  +  Ca(nu)*nu )
//     with added-mass matrix Ma, Coriolis matrix Ca(nu) and linear/quadratic
//     damping D(nu), all expressed in the body frame;
//   * optional periodic horizontal force (square wave) for excitation tests.
//
// Neutral buoyancy is obtained by choosing fluid_density = m / V.
//
// SDF usage (inside a <model>):
//   <plugin filename="libunderwater_object_system.so"
//           name="underwater_object_system">
//     <link_name>base_link</link_name>
//     <fluid_density>954.93</fluid_density>
//     <volume>0.5235987755982988</volume>
//     <radius>0.5</radius>
//     <slices>200</slices>
//     <added_mass>269.1 269.1 269.1 0 0 0</added_mass>
//     <linear_damping>0 0 0 0 0 0</linear_damping>
//     <quadratic_damping>200 200 200 20 20 20</quadratic_damping>
//     <horizontal_force_amplitude>500</horizontal_force_amplitude>
//     <horizontal_force_period>5</horizontal_force_period>
//     <debug>0</debug>
//   </plugin>

#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <sdf/Element.hh>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace simulator
{
namespace
{
constexpr std::size_t kDOF = 6;

// Multiply a 6x6 matrix (row-major, 36 doubles) by a 6-vector.
inline void MatVec(const double (&m)[36], const double (&v)[6],
                   double (&out)[6])
{
  for (std::size_t i = 0; i < 6; ++i)
  {
    out[i] = 0.0;
    for (std::size_t j = 0; j < 6; ++j)
      out[i] += m[i * 6 + j] * v[j];
  }
}

inline void SkewOf(const ignition::math::Vector3d &w, double (&s)[3][3])
{
  s[0][0] = 0.0;   s[0][1] = -w.Z(); s[0][2] =  w.Y();
  s[1][0] =  w.Z(); s[1][1] = 0.0;   s[1][2] = -w.X();
  s[2][0] = -w.Y(); s[2][1] =  w.X(); s[2][2] = 0.0;
}
}  // namespace

class UnderwaterObjectSystemPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
  // Documentation inherited
  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &) override
  {
    this->model = _entity;

	//Вычитываем тип модели mesh box sphere 
// 

    if (_sdf->HasElement("link_name"))
      this->linkName = _sdf->Get<std::string>("link_name");
    if (_sdf->HasElement("fluid_density"))
      this->fluidDensity = _sdf->Get<double>("fluid_density");
    if (_sdf->HasElement("volume"))
      this->volume = _sdf->Get<double>("volume");
    if (_sdf->HasElement("radius"))
      this->radius = _sdf->Get<double>("radius");
    if (_sdf->HasElement("slices"))
      this->slices = static_cast<unsigned int>(_sdf->Get<int>("slices"));
    if (_sdf->HasElement("debug"))
      this->debug = _sdf->Get<bool>("debug");

    // ---- Fossen hydrodynamic model parameters (as in HydrodynamicModel) ----
    this->LoadVecParam(_sdf, "added_mass", 36, this->addedMass);
    this->LoadVecParam(_sdf, "linear_damping", 6, this->linearDamping);
    this->LoadVecParam(_sdf, "linear_damping_forward_speed", 6,
        this->linearDampingForwardSpeed);
    this->LoadVecParam(_sdf, "quadratic_damping", 6, this->quadraticDamping);

    if (_sdf->HasElement("added_mass_acceleration"))
      this->useAddedMassAcceleration =
          _sdf->Get<bool>("added_mass_acceleration");
    if (_sdf->HasElement("horizontal_force_amplitude"))
      this->horizontalForceAmplitude =
          _sdf->Get<double>("horizontal_force_amplitude");
    if (_sdf->HasElement("horizontal_force_period"))
      this->horizontalForcePeriod =
          _sdf->Get<double>("horizontal_force_period");
    if (_sdf->HasElement("cmd_force_gain"))
      this->cmdForceGain = _sdf->Get<double>("cmd_force_gain"); if (_sdf->HasElement("cmd_torque_gain"))
      this->cmdTorqueGain = _sdf->Get<double>("cmd_torque_gain");
    if (_sdf->HasElement("cmd_timeout"))
      this->cmdTimeout = _sdf->Get<double>("cmd_timeout");
    if (_sdf->HasElement("cmd_mode"))
      this->cmdMode = _sdf->Get<std::string>("cmd_mode");
    if (_sdf->HasElement("cmd_velocity_gain"))
      this->cmdVelocityGain = _sdf->Get<double>("cmd_velocity_gain");
    if (_sdf->HasElement("cmd_vel_kp"))
      this->cmdVelKp = _sdf->Get<double>("cmd_vel_kp");
    if (_sdf->HasElement("cmd_ang_kp"))
      this->cmdAngKp = _sdf->Get<double>("cmd_ang_kp");

    // Derive missing geometry parameters from the sphere volume/radius
    if (this->radius <= 0.0 && this->volume > 0.0)
      this->radius = std::cbrt(3.0 * this->volume / (4.0 * kPi));
    if (this->volume <= 0.0 && this->radius > 0.0)
      this->volume = 4.0 / 3.0 * kPi
          * this->radius * this->radius * this->radius;
    this->ComputeSlices();

    // World gravity (defaults to -9.8 m/s^2 along z)
    const auto world = _ecm.EntityByComponents(
        ignition::gazebo::components::World());
    const auto gravityComp =
        _ecm.Component<ignition::gazebo::components::Gravity>(world);
    if (gravityComp)
      this->gravity = gravityComp->Data();

    // Model and link entities
    const auto modelNameComp =
        _ecm.Component<ignition::gazebo::components::Name>(this->model);
    if (modelNameComp)
      this->modelName = modelNameComp->Data();
    for (const auto entity :
        _ecm.EntitiesByComponents(ignition::gazebo::components::Link()))
    {
      const auto parent =
          _ecm.Component<ignition::gazebo::components::ParentEntity>(entity);
      const auto name =
          _ecm.Component<ignition::gazebo::components::Name>(entity);
      if (parent && parent->Data() == this->model &&
          name && name->Data() == this->linkName)
      {
        this->link = entity;
        break;
      }
    }

    if (this->link == ignition::gazebo::kNullEntity)
    {
      ignwarn << "[UnderwaterObjectSystem] link '" << this->linkName
              << "' not found in model " << _entity << std::endl;
      return;
    }

    // Teleop command topic: <model>/cmd_vel (gz transport). Bridge it from
    // ROS 2 with: ros2 run ros_gz_bridge parameter_bridge \
    //   /<model>/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist

    this->cmdTopic = this->modelName + "/cmd_vel";
    this->gzNode.Subscribe(this->cmdTopic,
        std::function<void(const ignition::msgs::Twist &)>(
        [this](const ignition::msgs::Twist &_msg)
        {
          std::lock_guard<std::mutex> lock(this->mutex);
          this->cmdLin[0] = _msg.linear().x();
          this->cmdLin[1] = _msg.linear().y();
          this->cmdLin[2] = _msg.linear().z();
          this->cmdAng[0] = _msg.angular().x();
          this->cmdAng[1] = _msg.angular().y();
          this->cmdAng[2] = _msg.angular().z();
          this->haveCmd = true;
          this->cmdReceived = true;
        }));

    ignmsg << "[UnderwaterObjectSystem] attached to link '"
           << this->linkName << "' of model '" << this->modelName
           << "', V=" << this->volume << " m^3, r=" << this->radius
           << " m, slices=" << this->slices
           << ", rho=" << this->fluidDensity << " kg/m^3"
           << ", cmd_vel topic '" << this->cmdTopic << "'"
           << ", debug=" << (this->debug ? "on" : "off") << std::endl;
  }

  // Documentation inherited
  public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (this->link == ignition::gazebo::kNullEntity)
      return;

    // Checks must be enabled so that physics publishes WorldPose/velocity
    // components and applies external forces.
    ignition::gazebo::Link gzLink(this->link);
    gzLink.EnableVelocityChecks(_ecm, true);
    gzLink.EnableAccelerationChecks(_ecm, true);

    const auto poseComp =
        _ecm.Component<ignition::gazebo::components::WorldPose>(this->link);
    if (!poseComp)
      return;  // physics has not started yet

    const auto linVelComp = _ecm.Component<
        ignition::gazebo::components::WorldLinearVelocity>(this->link);
    const auto angVelComp = _ecm.Component<
        ignition::gazebo::components::WorldAngularVelocity>(this->link);
    ignition::math::Vector3d worldLinVel(0, 0, 0);
    ignition::math::Vector3d worldAngVel(0, 0, 0);
    if (linVelComp)
      worldLinVel = linVelComp->Data();
    if (angVelComp)
      worldAngVel = angVelComp->Data();

    // World -> body transformation
    const ignition::math::Quaterniond rot = poseComp->Data().Rot();
    const ignition::math::Vector3d bodyLin =
        rot.RotateVectorReverse(worldLinVel);
    const ignition::math::Vector3d bodyAng =
        rot.RotateVectorReverse(worldAngVel);

    double nu[6] = {bodyLin.X(), bodyLin.Y(), bodyLin.Z(),
                    bodyAng.X(), bodyAng.Y(), bodyAng.Z()};

    // ---- Fossen hydrodynamic forces (body frame) ---------------------------
    // Damping matrix D(nu): linear + forward-speed + quadratic terms
    double dNu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 6; ++i)
    {
      const std::size_t d = i * 6 + i;  // diagonal entry
      const double uAbs = std::fabs(nu[0]);
      dNu[i] = -(this->linearDamping[d] +
                 uAbs * this->linearDampingForwardSpeed[d] +
                 this->quadraticDamping[d] * std::fabs(nu[i])) * nu[i];
    }

    // Filtered body acceleration (numerical derivative, alpha = 0.05)
    const double tSec = std::chrono::duration<double>(_info.simTime).count();
    double nuDot[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (this->haveLastVel && tSec > this->lastTime)
    {
      const double dt = tSec - this->lastTime;
      const double alpha = 0.05;
      for (std::size_t i = 0; i < 6; ++i)
      {
        double raw = (nu[i] - this->lastNu[i]) / dt;
        raw = std::max(-50.0, std::min(50.0, raw));
        this->filteredNuDot[i] =
            (1.0 - alpha) * this->filteredNuDot[i] + alpha * raw;
        nuDot[i] = this->filteredNuDot[i];
      }
    }
    this->haveLastVel = true;
    this->lastTime = tSec;
    for (std::size_t i = 0; i < 6; ++i)
      this->lastNu[i] = nu[i];

    // Added-mass force: -Ma * nu_dot (optional: explicit acceleration
    // feedback can be numerically unstable at 1 kHz, see classic TODO in
    // HydrodynamicModel.cpp, so it is disabled by default)
    double maNuDot[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (this->useAddedMassAcceleration)
      MatVec(this->addedMass, nuDot, maNuDot);

    // Added-mass Coriolis matrix Ca(nu) (Fossen 2011, eq. 6.43):
    //   ab = Ma * nu;  Sa = -S(ab_lin)
    //   Ca = [ 0  Sa ; Sa  -S(ab_ang) ]
    double ab[6];
    MatVec(this->addedMass, nu, ab);
    double sa[3][3];
    SkewOf(ignition::math::Vector3d(ab[0], ab[1], ab[2]), sa);
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j)
        sa[i][j] = -sa[i][j];
    double sAng[3][3];
    SkewOf(ignition::math::Vector3d(ab[3], ab[4], ab[5]), sAng);
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j)
        sAng[i][j] = -sAng[i][j];

    // cor = -Ca * nu
    double cor[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 3; ++i)
    {
      for (std::size_t j = 0; j < 3; ++j)
        cor[i] -= sa[i][j] * nu[3 + j];
      for (std::size_t j = 0; j < 3; ++j)
        cor[i + 3] -= sa[i][j] * nu[j];
      for (std::size_t j = 0; j < 3; ++j)
        cor[i + 3] -= sAng[i][j] * nu[3 + j];
    }

    // tau_body = -(D nu) + cor - Ma*nu_dot;  dNu already contains the
    // negative drag force, so it is added as-is
    double tau[6];
    for (std::size_t i = 0; i < 6; ++i)
      tau[i] = dNu[i] - maNuDot[i] + cor[i];

    const ignition::math::Vector3d tauForceBody(tau[0], tau[1], tau[2]);
    const ignition::math::Vector3d tauTorqueBody(tau[3], tau[4], tau[5]);

    // Body -> world
    ignition::math::Vector3d worldForce = rot * tauForceBody;
    ignition::math::Vector3d worldTorque = rot * tauTorqueBody;

    // ---- Buoyancy (Archimedes, slice based, water surface at z = 0) --------
    double submergedVolume = 0.0;
    const double linkZ = poseComp->Data().Pos().Z();
    for (std::size_t i = 0; i < this->sliceVolumes.size(); ++i)
    {
      const double centerWorldZ = linkZ +
          (rot * ignition::math::Vector3d(0, 0, this->sliceCenters[i])).Z();
      if (centerWorldZ < 0.0)
        submergedVolume += this->sliceVolumes[i];
    }
    const ignition::math::Vector3d buoyancyWorld =
        -this->gravity * (this->fluidDensity * submergedVolume);
    worldForce += buoyancyWorld;

    // ---- Optional periodic horizontal excitation ----------------------------
    // Square wave: force ON for `horizontal_force_period` seconds, then OFF
    // for the same duration (full cycle 2 * period)
    if (this->horizontalForceAmplitude != 0.0 &&
        std::fmod(tSec, 2.0 * this->horizontalForcePeriod) <
            this->horizontalForcePeriod)
    {
      worldForce.X() += this->horizontalForceAmplitude;
    }

    // ---- Teleop command (gz topic <model>/cmd_vel, bridged from ROS 2) ------
    double cl[3] = {0.0, 0.0, 0.0};
    double ca[3] = {0.0, 0.0, 0.0};
    bool haveCmd = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->cmdReceived)
      {
        this->lastCmdTime = tSec;
        this->cmdReceived = false;
      }
      haveCmd = this->haveCmd;
      // If no twist arrived for cmd_timeout seconds (teleop closed, key
      // released), treat the desired velocity as zero so the body actively
      // brakes to a stop instead of drifting.
      if (haveCmd && (tSec - this->lastCmdTime) > this->cmdTimeout)
      {
        for (std::size_t i = 0; i < 3; ++i)
        {
          this->cmdLin[i] = 0.0;
          this->cmdAng[i] = 0.0;
        }
      }
      for (std::size_t i = 0; i < 3; ++i)
      {
        cl[i] = this->cmdLin[i];
        ca[i] = this->cmdAng[i];
      }
    }
    if (haveCmd)
    {
      if (this->cmdMode == "force")
      {
        // Force mode: twist is a force/torque command in the body frame
        const ignition::math::Vector3d cmdForceBody(
            cl[0] * this->cmdForceGain,
            cl[1] * this->cmdForceGain,
            cl[2] * this->cmdForceGain);
        const ignition::math::Vector3d cmdTorqueBody(
            ca[0] * this->cmdTorqueGain,
            ca[1] * this->cmdTorqueGain,
            ca[2] * this->cmdTorqueGain);
        worldForce += rot * cmdForceBody;
        worldTorque += rot * cmdTorqueBody;
      }
      else
      {
        // Velocity mode (default): twist is the desired body velocity;
        // a P controller drives the body to it. A zero twist therefore
        // actively brakes the body to a full stop.
        const ignition::math::Vector3d velDesired(
            cl[0] * this->cmdVelocityGain,
            cl[1] * this->cmdVelocityGain,
            cl[2] * this->cmdVelocityGain);
        const ignition::math::Vector3d angDesired(ca[0], ca[1], ca[2]);
        const ignition::math::Vector3d velErr = velDesired - bodyLin;
        const ignition::math::Vector3d angErr = angDesired - bodyAng;
        const ignition::math::Vector3d cmdForceBody =
            velErr * this->cmdVelKp;
        const ignition::math::Vector3d cmdTorqueBody =
            angErr * this->cmdAngKp;
        worldForce += rot * cmdForceBody;
        worldTorque += rot * cmdTorqueBody;
      }
    }

    gzLink.AddWorldWrench(_ecm, worldForce, worldTorque);

    if (this->debug && ++this->debugCounter % 100 == 0)
    {
      ignmsg << "[UnderwaterObjectSystem][debug] t=" << _info.simTime.count()
             << " ns, V_sub=" << submergedVolume
             << " m^3, F_hydro=(" << tauForceBody.X() << ", "
             << tauForceBody.Y() << ", " << tauForceBody.Z()
             << ") N body, F_b=(" << buoyancyWorld.X() << ", "
             << buoyancyWorld.Y() << ", " << buoyancyWorld.Z() << ") N"
             << ", v_body=(" << bodyLin.X() << ", " << bodyLin.Y()
             << ", " << bodyLin.Z() << ")"
             << ", cmd=(" << cmdLin[0] << ", " << cmdLin[1] << ", "
             << cmdLin[2] << "; " << cmdAng[0] << ", " << cmdAng[1]
             << ", " << cmdAng[2] << ")"
             << std::endl;
    }
  }

  // Documentation inherited
  private: void ComputeSlices()
  {
    this->sliceVolumes.clear();
    this->sliceCenters.clear();
    if (this->slices < 1)
      this->slices = 1;

    const double dz = 2.0 * this->radius / this->slices;
    double total = 0.0;
    for (unsigned int i = 0; i < this->slices; ++i)
    {
      const double z0 = -this->radius + i * dz;
      const double z1 = z0 + dz;
      // Sphere segment volume between horizontal planes z0 and z1:
      // V = pi * (R^2 * (z1-z0) - (z1^3 - z0^3) / 3)
      const double segVol = kPi * (this->radius * this->radius *
          (z1 - z0) - (z1 * z1 * z1 - z0 * z0 * z0) / 3.0);
      this->sliceVolumes.push_back(segVol);
      this->sliceCenters.push_back((z0 + z1) / 2.0);
      total += segVol;
    }

    if (total > 0.0 && std::abs(total - this->volume) > 1e-9)
    {
      const double factor = this->volume / total;
      for (auto &v : this->sliceVolumes)
        v *= factor;
    }
  }

  private: void LoadVecParam(const std::shared_ptr<const sdf::Element> &_sdf,
                             const std::string &_tag,
                             std::size_t _size, double (&_out)[36])
  {
    for (std::size_t i = 0; i < 36; ++i)
      _out[i] = 0.0;
    if (!_sdf->HasElement(_tag))
      return;

    std::vector<double> values;
    std::istringstream ss(_sdf->Get<std::string>(_tag));
    double v = 0.0;
    while (ss >> v)
      values.push_back(v);

    if (values.size() >= 36)
    {
      // Full 6x6 matrix (row-major)
      for (std::size_t i = 0; i < 36; ++i)
        _out[i] = values[i];
    }
    else if (values.size() >= 6)
    {
      // Diagonal matrix from the first 6 coefficients
      for (std::size_t i = 0; i < 6; ++i)
        _out[i * 6 + i] = values[i];
    }
    else if (!values.empty())
    {
      // One scalar applied to the whole diagonal
      const double c = values[0];
      for (std::size_t i = 0; i < 6; ++i)
        _out[i * 6 + i] = c;
    }
  }

  private: static constexpr double kPi = 3.14159265358979323846;

  private: ignition::gazebo::Entity model =
      ignition::gazebo::kNullEntity;
  private: ignition::gazebo::Entity link =
      ignition::gazebo::kNullEntity;
  private: std::string linkName{"base_link"};
  private: std::string modelName{"model"};

  private: double fluidDensity{1028.0};
  private: double volume{0.0};
  private: double radius{0.5};
  private: unsigned int slices{200};
  private: bool enabled{true};
  private: bool useAddedMassAcceleration{false};
  private: bool debug{false};
  private: unsigned int debugCounter{0};

  private: std::vector<double> sliceVolumes;
  private: std::vector<double> sliceCenters;

  // Fossen coefficients (36 = full 6x6, 6 = diagonal)
  private: double addedMass[36] = {0.0};
  private: double linearDamping[36] = {0.0};
  private: double linearDampingForwardSpeed[36] = {0.0};
  private: double quadraticDamping[36] = {0.0};

  // Numerical acceleration filter state
  private: bool haveLastVel{false};
  private: double lastTime{0.0};
  private: double lastNu[6] = {0.0};
  private: double filteredNuDot[6] = {0.0};

  // Periodic horizontal force
  private: double horizontalForceAmplitude{0.0};
  private: double horizontalForcePeriod{5.0};

  // Teleop command (cmd_vel)
  private: std::string cmdTopic;
  private: bool haveCmd{false};
  private: double cmdLin[3] = {0.0, 0.0, 0.0};
  private: double cmdAng[3] = {0.0, 0.0, 0.0};
  private: double cmdForceGain{500.0};
  private: double cmdTorqueGain{100.0};
  private: double cmdTimeout{0.5};
  private: bool cmdReceived{false};
  private: double lastCmdTime{0.0};

  // Command mode: "velocity" (twist = desired velocity, zero = stop)
  // or "force" (twist = body-frame force/torque)
  private: std::string cmdMode{"velocity"};
  private: double cmdVelocityGain{1.0};
  private: double cmdVelKp{600.0};
  private: double cmdAngKp{100.0};

  private: ignition::transport::Node gzNode;
  private: std::mutex mutex;

  private: ignition::math::Vector3d gravity{0, 0, -9.8};
};
}

IGNITION_ADD_PLUGIN(
    simulator::UnderwaterObjectSystemPlugin,
    ignition::gazebo::System,
    simulator::UnderwaterObjectSystemPlugin::ISystemConfigure,
    simulator::UnderwaterObjectSystemPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    simulator::UnderwaterObjectSystemPlugin, "underwater_object_system")
