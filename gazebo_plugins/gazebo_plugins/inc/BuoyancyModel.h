#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <IBuoyancyModel.h>

// Buoyancy (Archimedes) model of a rigid body.
//
// All parameters are provided once through the constructor and never change
// afterwards; ComputeRestoringVector() is const.
//
// Physics:
//   * Archimedes force acting in the buoyancy centre:
//       F_b = -fluidDensity * volume * gravity
//   * the body mass is derived from the volume and the fluid density
//     (neutral buoyancy, m = fluidDensity * volume), so the weight
//       F_g = +m * gravity = -F_b
//     acts in the mass centre.
//
// The returned 6x1 vector is [F_b; M] with
//   M = buoyancyCenter x F_b + massCenter x F_g
// (both moments are taken about the link origin).
class BuoyancyModel : public IBuoyancyModel {
public:
  BuoyancyModel(const double volume,
                const double fluidDensity,
                Eigen::Vector3d &&buoyancyCenter,
                Eigen::Vector3d &&massCenter,
                Eigen::Vector3d &&gravity);
  ~BuoyancyModel() override;

  Eigen::Matrix<double, 6, 1> ComputeRestoringVector() const override;

private:
  const double m_volume;
  const double m_fluidDensity;
  const Eigen::Vector3d m_buoyancyCenter;
  const Eigen::Vector3d m_massCenter;
  const Eigen::Vector3d m_gravity;
  const unsigned int m_slicesCount;
};
