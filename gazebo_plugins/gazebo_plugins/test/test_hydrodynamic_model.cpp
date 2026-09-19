#include <HydrodynamicModel.h>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

constexpr double kTolerance = 1e-9;

// ---- Reference (independent) implementation of the Fossen terms -----------

Matrix3d Skew(const Vector3d &v) {
  Matrix3d s;
  s << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return s;
}

Vector6d RefDampingForce(const Vector6d &lin, const Vector6d &linFwd,
                         const Vector6d &quad, const Vector6d &nu) {
  Vector6d force = Vector6d::Zero();
  for (int i = 0; i < 6; ++i) {
    const double coefficient = lin(i) + linFwd(i) * std::abs(nu(0)) +
                               quad(i) * std::abs(nu(i));
    force(i) = -coefficient * nu(i);
  }
  return force;
}

Vector6d RefInertiaForce(const Matrix6d &mass, const Vector6d &nuDot) {
  return -mass * nuDot;
}

// General Fossen Coriolis expression (valid for rigid-body and added mass):
//   C = [      0              -S(M11*v1 + M12*v2) ]
//       [ -S(M11*v1 + M12*v2)  -S(M21*v1 + M22*v2) ]
Vector6d RefCoriolisForce(const Matrix6d &mass, const Vector6d &nu) {
  const Vector3d v1 = nu.head<3>();
  const Vector3d v2 = nu.tail<3>();

  const Matrix3d m11 = mass.block<3, 3>(0, 0);
  const Matrix3d m12 = mass.block<3, 3>(0, 3);
  const Matrix3d m21 = mass.block<3, 3>(3, 0);
  const Matrix3d m22 = mass.block<3, 3>(3, 3);

  const Matrix3d upper = -Skew(m11 * v1 + m12 * v2);
  const Matrix3d lowerRight = -Skew(m21 * v1 + m22 * v2);

  Matrix6d coriolis = Matrix6d::Zero();
  coriolis.block<3, 3>(0, 3) = upper;
  coriolis.block<3, 3>(3, 0) = upper;
  coriolis.block<3, 3>(3, 3) = lowerRight;
  return -coriolis * nu;
}

const Matrix6d kZeroMass = Matrix6d::Zero();
const Vector6d kZero6 = Vector6d::Zero();

// ---- Tests ----------------------------------------------------------------

TEST(HydrodynamicModelInitialization, ZeroParametersGiveZeroWrench) {
  const HydrodynamicModel model(kZeroMass, kZero6, kZero6, kZero6);

  Vector6d nu;
  nu << 1.0, -2.0, 3.0, 0.4, -0.5, 0.6;
  Vector6d nuDot;
  nuDot << -0.1, 0.2, -0.3, 0.7, -0.8, 0.9;

  EXPECT_TRUE(model.ComputeHydrodynamicForces(nu, nuDot).isZero(kTolerance));
  EXPECT_TRUE(
      model.ComputeHydrodynamicForces(kZero6, kZero6).isZero(kTolerance));

  // Zero velocity and acceleration: no wrench even for a non-trivial mass.
  Matrix6d mass = Matrix6d::Zero();
  mass.diagonal() << 750.0, 750.0, 750.0, 50.0, 50.0, 50.0;
  const HydrodynamicModel stuck(mass, kZero6, kZero6, kZero6);
  EXPECT_TRUE(
      stuck.ComputeHydrodynamicForces(kZero6, kZero6).isZero(kTolerance));
}

// Damping component, isolated: zero mass matrix.
TEST(HydrodynamicModelDamping, LinearDampingIsDiagonalAndOpposesMotion) {
  Vector6d lin;
  lin << 10.0, 20.0, 30.0, 1.0, 2.0, 3.0;
  const HydrodynamicModel model(kZeroMass, lin, kZero6, kZero6);

  Vector6d nu;
  nu << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  const Vector6d expected = RefDampingForce(lin, kZero6, kZero6, nu);
  const Vector6d actual = model.ComputeHydrodynamicForces(nu, kZero6);
  EXPECT_TRUE(actual.isApprox(expected, kTolerance));

  // Explicit numbers: -diag(lin) * nu
  EXPECT_NEAR(actual(0), -10.0, kTolerance);
  EXPECT_NEAR(actual(1), -40.0, kTolerance);
  EXPECT_NEAR(actual(2), -90.0, kTolerance);
  EXPECT_NEAR(actual(3), -4.0, kTolerance);
  EXPECT_NEAR(actual(4), -10.0, kTolerance);
  EXPECT_NEAR(actual(5), -18.0, kTolerance);

  // Doubling the coefficients doubles the damping force (initialization).
  const HydrodynamicModel doubled(kZeroMass, 2.0 * lin, kZero6, kZero6);
  EXPECT_TRUE(
      doubled.ComputeHydrodynamicForces(nu, kZero6).isApprox(2.0 * expected,
                                                             kTolerance));
}

TEST(HydrodynamicModelDamping, QuadraticDampingScalesWithAbsVelocity) {
  Vector6d quad;
  quad << 0.5, 1.0, 1.5, 2.0, 2.5, 3.0;
  const HydrodynamicModel model(kZeroMass, kZero6, kZero6, quad);

  Vector6d nu;
  nu << 2.0, -2.0, 4.0, -4.0, 1.0, -1.0;

  const Vector6d expected = RefDampingForce(kZero6, kZero6, quad, nu);
  EXPECT_TRUE(model.ComputeHydrodynamicForces(nu, kZero6)
                  .isApprox(expected, kTolerance));

  EXPECT_NEAR(expected(0), -0.5 * 2.0 * 2.0, kTolerance);
  EXPECT_NEAR(expected(1), 1.0 * 2.0 * 2.0, kTolerance);
}

TEST(HydrodynamicModelDamping, ForwardSpeedTermDependsOnSurgeSpeed) {
  // Surge coefficient is zero so that the |u| dependence is visible without
  // the sign of the force following the sign of u for that component.
  Vector6d linFwd;
  linFwd << 0.0, 4.0, 4.0, 0.5, 0.5, 0.5;
  const HydrodynamicModel model(kZeroMass, kZero6, linFwd, kZero6);

  Vector6d nu;
  nu << 2.0, 1.0, -1.0, 1.0, -1.0, 0.0;

  const Vector6d actual = model.ComputeHydrodynamicForces(nu, kZero6);
  EXPECT_TRUE(
      actual.isApprox(RefDampingForce(kZero6, linFwd, kZero6, nu), kTolerance));

  // Explicit: for i=1, coefficient = linFwd(1) * |u| = 4 * 2 = 8
  EXPECT_NEAR(actual(1), -8.0 * nu(1), kTolerance);

  // Reversing the surge speed keeps the same force (only |u| is used).
  Vector6d nuNegative = nu;
  nuNegative(0) = -nu(0);
  EXPECT_TRUE(model.ComputeHydrodynamicForces(nuNegative, kZero6)
                  .isApprox(actual, kTolerance));

  // Doubling the surge speed doubles the forward-speed contribution.
  Vector6d nuFaster = nu;
  nuFaster(0) = 2.0 * nu(0);
  EXPECT_NEAR(model.ComputeHydrodynamicForces(nuFaster, kZero6)(1),
              -16.0 * nu(1), kTolerance);
}

// Inertia component (rigid body + added mass in one matrix), isolated: zero
// velocity and zero damping.
TEST(HydrodynamicModelInertia, MultipliesFullMassMatrixWithAcceleration) {
  Matrix6d mass = Matrix6d::Zero();
  mass << 250.0, 10.0, 0.0, 0.0, 0.0, 0.0,
          10.0, 300.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 400.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 20.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 25.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 30.0;
  const HydrodynamicModel model(mass, kZero6, kZero6, kZero6);

  Vector6d nuDot;
  nuDot << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;

  const Vector6d actual = model.ComputeHydrodynamicForces(kZero6, nuDot);
  EXPECT_TRUE(actual.isApprox(RefInertiaForce(mass, nuDot), kTolerance));

  // Explicit off-diagonal contribution: -(250*1 + 10*2) = -270
  EXPECT_NEAR(actual(0), -270.0, kTolerance);
  EXPECT_NEAR(actual(1), -(10.0 * 1.0 + 300.0 * 2.0), kTolerance);

  // Doubling the mass matrix doubles the force.
  const HydrodynamicModel doubled(2.0 * mass, kZero6, kZero6, kZero6);
  EXPECT_TRUE(doubled.ComputeHydrodynamicForces(kZero6, nuDot)
                  .isApprox(2.0 * actual, kTolerance));
}

// The general Coriolis expression must reproduce both legacy special cases.
TEST(HydrodynamicModelCoriolis, ReproducesAddedMassCoriolisEq643) {
  // M = M_A (diagonal): legacy added-mass Coriolis (Fossen eq. 6.43)
  Matrix6d addedMass = Matrix6d::Zero();
  addedMass.diagonal() << 250.0, 250.0, 250.0, 0.0, 0.0, 0.0;
  const HydrodynamicModel model(addedMass, kZero6, kZero6, kZero6);

  Vector6d nu;
  nu << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  const Vector6d actual = model.ComputeHydrodynamicForces(nu, kZero6);

  const Vector6d explicitExpected =
      (Vector6d() << 0.0, 0.0, 250.0, 0.0, 0.0, 0.0).finished();
  EXPECT_TRUE(actual.isApprox(explicitExpected, kTolerance));
  EXPECT_TRUE(actual.isApprox(RefCoriolisForce(addedMass, nu), kTolerance));

  // General non-diagonal added-mass matrix.
  Matrix6d generalMass = Matrix6d::Zero();
  generalMass << 250.0, 30.0, 0.0, 0.0, 0.0, 10.0,
                 30.0, 300.0, 0.0, 0.0, 5.0, 0.0,
                 0.0, 0.0, 400.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 20.0, 0.0, 0.0,
                 0.0, 5.0, 0.0, 0.0, 25.0, 0.0,
                 10.0, 0.0, 0.0, 0.0, 0.0, 30.0;
  const HydrodynamicModel general(generalMass, kZero6, kZero6, kZero6);
  Vector6d nuGeneral;
  nuGeneral << 1.0, -2.0, 3.0, 0.4, -0.5, 0.6;
  EXPECT_TRUE(general.ComputeHydrodynamicForces(nuGeneral, kZero6)
                  .isApprox(RefCoriolisForce(generalMass, nuGeneral),
                            kTolerance));
}

TEST(HydrodynamicModelCoriolis, ReproducesRigidBodyCoriolisEq357) {
  // M = diag(m*I, I_b): legacy rigid-body Coriolis (Fossen eq. 3.57)
  const double bodyMass = 2.0;
  Matrix6d rigidMass = Matrix6d::Zero();
  rigidMass.diagonal() << bodyMass, bodyMass, bodyMass, 3.0, 4.0, 5.0;
  const HydrodynamicModel model(rigidMass, kZero6, kZero6, kZero6);

  // Pure surge + pitch rate: expected force = (0, 0, m*u*q, 0, 0, 0)
  Vector6d nu;
  nu << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  const Vector6d actual = model.ComputeHydrodynamicForces(nu, kZero6);

  const Vector6d explicitExpected =
      (Vector6d() << 0.0, 0.0, bodyMass * 1.0 * 1.0, 0.0, 0.0, 0.0)
          .finished();
  EXPECT_TRUE(actual.isApprox(explicitExpected, kTolerance));
  EXPECT_TRUE(actual.isApprox(RefCoriolisForce(rigidMass, nu), kTolerance));

  // Zero velocity -> no Coriolis force.
  EXPECT_TRUE(
      model.ComputeHydrodynamicForces(kZero6, kZero6).isZero(kTolerance));

  // Doubling the mass matrix doubles the Coriolis force.
  Vector6d nuGeneral;
  nuGeneral << 1.0, -2.0, 3.0, 0.4, -0.5, 0.6;
  const HydrodynamicModel heavier(2.0 * rigidMass, kZero6, kZero6, kZero6);
  EXPECT_TRUE(heavier.ComputeHydrodynamicForces(nuGeneral, kZero6)
                  .isApprox(2.0 * model.ComputeHydrodynamicForces(nuGeneral,
                                                                  kZero6),
                            kTolerance));
}

// A full mass matrix with off-diagonal blocks couples all terms.
TEST(HydrodynamicModelCoriolis, HandlesFullCouplingBlocks) {
  Matrix6d mass = Matrix6d::Zero();
  mass << 500.0, 0.0, 0.0, 0.0, 50.0, 0.0,
          0.0, 500.0, 0.0, -50.0, 0.0, 0.0,
          0.0, 0.0, 500.0, 0.0, 0.0, 20.0,
          0.0, -50.0, 0.0, 60.0, 0.0, 0.0,
          50.0, 0.0, 0.0, 0.0, 70.0, 0.0,
          0.0, 0.0, 20.0, 0.0, 0.0, 80.0;
  const HydrodynamicModel model(mass, kZero6, kZero6, kZero6);

  Vector6d nu;
  nu << 1.0, -2.0, 3.0, 0.4, -0.5, 0.6;
  EXPECT_TRUE(model.ComputeHydrodynamicForces(nu, kZero6)
                  .isApprox(RefCoriolisForce(mass, nu), kTolerance));
}

// Superposition: the public method returns the sum of all Fossen components.
TEST(HydrodynamicModelComputation, IsSumOfAllFossenComponents) {
  Matrix6d mass = Matrix6d::Zero();
  mass << 750.0, 0.0, 0.0, 0.0, 40.0, 0.0,
          0.0, 750.0, 0.0, -40.0, 0.0, 0.0,
          0.0, 0.0, 750.0, 0.0, 0.0, 15.0,
          0.0, -40.0, 0.0, 50.0, 0.0, 0.0,
          40.0, 0.0, 0.0, 0.0, 55.0, 0.0,
          0.0, 0.0, 15.0, 0.0, 0.0, 60.0;
  Vector6d lin;
  lin << 10.0, 20.0, 30.0, 1.0, 2.0, 3.0;
  Vector6d linFwd;
  linFwd << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  Vector6d quad;
  quad << 0.5, 0.5, 0.5, 0.1, 0.1, 0.1;

  const HydrodynamicModel model(mass, lin, linFwd, quad);

  Vector6d nu;
  nu << 1.0, -2.0, 3.0, 0.4, -0.5, 0.6;
  Vector6d nuDot;
  nuDot << 0.2, -0.1, 0.3, -0.2, 0.1, 0.4;

  const Vector6d expected = RefDampingForce(lin, linFwd, quad, nu) +
                            RefInertiaForce(mass, nuDot) +
                            RefCoriolisForce(mass, nu);
  EXPECT_TRUE(model.ComputeHydrodynamicForces(nu, nuDot)
                  .isApprox(expected, kTolerance));

  // The method is const: repeated calls give the same value.
  EXPECT_TRUE(model.ComputeHydrodynamicForces(nu, nuDot)
                  .isApprox(model.ComputeHydrodynamicForces(nu, nuDot)));
}

// The interface allows substituting mock implementations.
class MockHydrodynamicModel : public IHydrodynamicModel {
public:
  explicit MockHydrodynamicModel(Vector6d value) : m_value(std::move(value)) {}
  Vector6d ComputeHydrodynamicForces(const Vector6d &, const Vector6d &)
      const override {
    return m_value;
  }
private:
  const Vector6d m_value;
};

TEST(HydrodynamicModelInterface, SupportsMockImplementation) {
  Vector6d expected;
  expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const std::unique_ptr<IHydrodynamicModel> mock =
      std::make_unique<MockHydrodynamicModel>(expected);
  EXPECT_TRUE(
      mock->ComputeHydrodynamicForces(kZero6, kZero6).isApprox(expected));
}

}  // namespace
