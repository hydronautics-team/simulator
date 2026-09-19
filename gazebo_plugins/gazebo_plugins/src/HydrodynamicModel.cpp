#include <HydrodynamicModel.h>

#include <cmath>

namespace {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

Matrix3d Skew(const Vector3d &v) {
  Matrix3d s;
  s << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return s;
}

}  // namespace

HydrodynamicModel::HydrodynamicModel(const Matrix6d &massMatrix,
                                     const Vector6d &linearDamping,
                                     const Vector6d &linearDampingForward,
                                     const Vector6d &quadraticDamping)
    : m_massMatrix(massMatrix),
      m_linearDamping(linearDamping),
      m_linearDampingForward(linearDampingForward),
      m_quadraticDamping(quadraticDamping) {}

HydrodynamicModel::~HydrodynamicModel() = default;

HydrodynamicModel::Matrix6d
HydrodynamicModel::ComputeDampingMatrix(const Vector6d &velocity) const {
  Matrix6d damping = Matrix6d::Zero();
  const double forwardSpeed = std::abs(velocity(0));
  for (int i = 0; i < 6; ++i) {
    damping(i, i) = m_linearDamping(i) +
                    m_linearDampingForward(i) * forwardSpeed +
                    m_quadraticDamping(i) * std::abs(velocity(i));
  }
  return damping;
}

HydrodynamicModel::Matrix6d
HydrodynamicModel::ComputeCoriolisMatrix(const Vector6d &velocity) const {
  // General Fossen Coriolis expression for a 6x6 mass matrix M:
  //   C(nu) = [      0             -S(M11*v1 + M12*v2) ]
  //           [ -S(M11*v1 + M12*v2)  -S(M21*v1 + M22*v2) ]
  // For M = M_A this is eq. 6.43; for M = diag(m*I, I_b) it is eq. 3.57.
  const Eigen::Vector3d v1 = velocity.head<3>();
  const Eigen::Vector3d v2 = velocity.tail<3>();

  const Matrix3d m11 = m_massMatrix.block<3, 3>(0, 0);
  const Matrix3d m12 = m_massMatrix.block<3, 3>(0, 3);
  const Matrix3d m21 = m_massMatrix.block<3, 3>(3, 0);
  const Matrix3d m22 = m_massMatrix.block<3, 3>(3, 3);

  const Matrix3d upper = -Skew(m11 * v1 + m12 * v2);
  const Matrix3d lowerRight = -Skew(m21 * v1 + m22 * v2);

  Matrix6d coriolis = Matrix6d::Zero();
  coriolis.block<3, 3>(0, 3) = upper;
  coriolis.block<3, 3>(3, 0) = upper;
  coriolis.block<3, 3>(3, 3) = lowerRight;
  return coriolis;
}

HydrodynamicModel::Vector6d
HydrodynamicModel::ComputeDampingForce(const Vector6d &velocity) const {
  return -ComputeDampingMatrix(velocity) * velocity;
}

HydrodynamicModel::Vector6d
HydrodynamicModel::ComputeInertiaForce(const Vector6d &acceleration) const {
  // Includes both the rigid-body and the added-mass inertia (single matrix).
  return -m_massMatrix * acceleration;
}

HydrodynamicModel::Vector6d
HydrodynamicModel::ComputeCoriolisForce(const Vector6d &velocity) const {
  return -ComputeCoriolisMatrix(velocity) * velocity;
}

HydrodynamicModel::Vector6d HydrodynamicModel::ComputeHydrodynamicForces(
    const Vector6d &velocity, const Vector6d &acceleration) const {
  return ComputeDampingForce(velocity) +
         ComputeInertiaForce(acceleration) +
         ComputeCoriolisForce(velocity);
}
