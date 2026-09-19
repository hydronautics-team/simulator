#pragma once

#include <eigen3/Eigen/Core>

// Interface for a buoyancy model. Kept deliberately small (a single public
// method) so that mock implementations can be substituted in tests / by the
// UnderwaterObjectPlugin facade.
class IBuoyancyModel {
public:
  virtual ~IBuoyancyModel() = default;

  // Returns the restoring wrench [force; moment] (6x1) in the link frame:
  //   linear  part - Archimedes force  F_b = -rho * V * g
  //   angular part - moments of the gravity and buoyancy forces about the
  //                  link origin: r_b x F_b + r_g x F_g
  virtual Eigen::Matrix<double, 6, 1> ComputeRestoringVector() const = 0;
};
