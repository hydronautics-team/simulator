#pragma once

#include <eigen3/Eigen/Core>

#include <IHydrodynamicModel.h>

// Hydrodynamic (Fossen) model of a rigid body moving in a fluid.
//
// All parameters are provided once through the constructor and never change
// afterwards. The model exposes a single public method
// ComputeHydrodynamicForces(velocity, acceleration); the individual Fossen
// terms are private implementation details.
//
// The mass is configured as a single 6x6 matrix which already includes the
// added mass:  M = M_RB + M_A  (as done in the legacy implementation, where
// both contributions enter the same skew-symmetric Coriolis expression).
//
//   tau = -( M * nu_dot  +  C_M(nu) * nu  +  D(nu) * nu )
//
// with (all in the body frame, 6 DOF):
//   M         - total mass matrix (rigid body + added mass)
//   C_M(nu)   - Coriolis matrix built from M:
//                 C = [      0            -S(M11*v1 + M12*v2) ]
//                     [ -S(M11*v1 + M12*v2)  -S(M21*v1 + M22*v2) ]
//               for M = [M11 M12; M21 M22]. This general form reproduces
//               both the rigid-body Coriolis (Fossen eq. 3.57, with
//               M = diag(m*I, I_b)) and the added-mass Coriolis
//               (Fossen eq. 6.43, with M = M_A).
//   D(nu)     - diagonal damping: diag( linearDamping
//                                      + linearDampingForward * |u|
//                                      + quadraticDamping * |nu_i| )
//
// (S(x) is the skew-symmetric cross-product operator.)
class HydrodynamicModel : public IHydrodynamicModel {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  HydrodynamicModel(const Matrix6d &massMatrix,
                    const Vector6d &linearDamping,
                    const Vector6d &linearDampingForward,
                    const Vector6d &quadraticDamping);
  ~HydrodynamicModel() override;

  // tau = -( M*nu_dot + C(nu)*nu + D(nu)*nu )
  Vector6d ComputeHydrodynamicForces(const Vector6d &velocity,
                                     const Vector6d &acceleration) const override;

private:
  Matrix6d ComputeDampingMatrix(const Vector6d &velocity) const;
  Matrix6d ComputeCoriolisMatrix(const Vector6d &velocity) const;

  Vector6d ComputeDampingForce(const Vector6d &velocity) const;
  Vector6d ComputeInertiaForce(const Vector6d &acceleration) const;
  Vector6d ComputeCoriolisForce(const Vector6d &velocity) const;

private:
  const Matrix6d m_massMatrix;
  const Vector6d m_linearDamping;
  const Vector6d m_linearDampingForward;
  const Vector6d m_quadraticDamping;
};
