#include <BuoyancyModel.h>

#include <utility>

BuoyancyModel::BuoyancyModel(const double volume,
                             const double fluidDensity,
                             Eigen::Vector3d &&buoyancyCenter,
                             Eigen::Vector3d &&massCenter,
                             Eigen::Vector3d &&gravity)
    : m_volume(volume),
      m_fluidDensity(fluidDensity),
      m_buoyancyCenter(std::move(buoyancyCenter)),
      m_massCenter(std::move(massCenter)),
      m_gravity(std::move(gravity)),
      m_slicesCount(200) {}

BuoyancyModel::~BuoyancyModel() = default;

Eigen::Matrix<double, 6, 1> BuoyancyModel::ComputeRestoringVector() const {
  // Mass of the displaced fluid; neutral buoyancy is assumed, so this is also
  // the mass of the body (m = fluidDensity * volume).
  const double displacedMass = m_fluidDensity * m_volume;

  // Archimedes force (opposite to the gravity vector) ...
  const Eigen::Vector3d buoyancyForce = -displacedMass * m_gravity;
  // ... and the weight of the body.
  const Eigen::Vector3d gravityForce = displacedMass * m_gravity;

  // Moments about the link origin.
  const Eigen::Vector3d buoyancyMoment = m_buoyancyCenter.cross(buoyancyForce);
  const Eigen::Vector3d gravityMoment = m_massCenter.cross(gravityForce);

  Eigen::Matrix<double, 6, 1> restoringVector;
  restoringVector << buoyancyForce, buoyancyMoment + gravityMoment;
  return restoringVector;
}
