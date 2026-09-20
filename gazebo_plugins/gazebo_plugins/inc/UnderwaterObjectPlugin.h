#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <BuoyancyModel.h>
#include <HydrodynamicModel.h>

// Facade that combines the buoyancy and hydrodynamic models for a body.
// The body state (pose and velocity) is read from the link components; the
// filtered acceleration is derived numerically from that velocity.
class UnderwaterObjectPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPreUpdate {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  UnderwaterObjectPlugin();
  ~UnderwaterObjectPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  // Numerical derivative of the body velocity with a low-pass filter
  Vector6d ComputeAcceleration(const Vector6d &_velocity, double _dtSeconds);

private:
  gz::sim::Model m_model;
  gz::sim::Link m_baseLink;
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
