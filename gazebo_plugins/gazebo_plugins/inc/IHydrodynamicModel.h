#pragma once

#include <eigen3/Eigen/Core>

// Interface for a hydrodynamic (Fossen) model. Kept small (a single public
// method) so that mock implementations can be substituted in tests / by the
// UnderwaterObjectPlugin facade.
//
// The facade is responsible for providing the body-frame velocity and the
// filtered acceleration; the model itself is a pure function of its
// constructor parameters.
class IHydrodynamicModel {
public:
  virtual ~IHydrodynamicModel() = default;

  // Returns the Fossen hydrodynamic wrench [force; moment] (6x1) in the body
  // frame for the given body velocity `velocity` and filtered acceleration
  // `acceleration`:
  //   tau = -( M*nu_dot + C(nu)*nu + D(nu)*nu )
  // where M is the total mass matrix (rigid body + added mass).
  virtual Eigen::Matrix<double, 6, 1> ComputeHydrodynamicForces(
      const Eigen::Matrix<double, 6, 1> &velocity,
      const Eigen::Matrix<double, 6, 1> &acceleration) const = 0;
};
