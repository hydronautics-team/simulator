#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>

#include <BuoyancyModel.h>
#include <HydrodynamicModel.h>

// Facade that combines the buoyancy and hydrodynamic models for a body.
// The body state (pose and velocity) is read from the link components; the
// filtered acceleration is derived numerically from that velocity.
class UnderwaterObjectPlugin : public ignition::gazebo::System,
                               public ignition::gazebo::ISystemConfigure,
                               public ignition::gazebo::ISystemPreUpdate {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  UnderwaterObjectPlugin();
  ~UnderwaterObjectPlugin() override;

  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  // Numerical derivative of the body velocity with a low-pass filter
  Vector6d ComputeAcceleration(const Vector6d &_velocity, double _dtSeconds);

private:
  ignition::gazebo::Model m_model;
  ignition::gazebo::Link m_baseLink;
  bool m_debugMode{false};
  unsigned int m_debugCounter{0};

  // Models (created once in Configure, never mutated afterwards)
  std::unique_ptr<BuoyancyModel> m_buoyancyModel;
  std::unique_ptr<HydrodynamicModel> m_hydroModel;

  // Velocity / acceleration filter state
  bool m_hasLastVelocity{false};
  double m_lastTimeSeconds{0.0};
  Vector6d m_lastVelocity{Vector6d::Zero()};
  Vector6d m_filteredAcceleration{Vector6d::Zero()};

  static constexpr double kAccelerationFilterAlpha = 0.05;
  static constexpr double kAccelerationClamp = 50.0;
};
