#include <BuoyancyModel.h>

#include <gtest/gtest.h>

#include <memory>

namespace {

constexpr double kGravity = -9.81;
constexpr double kTolerance = 1e-9;

// Verifies that every constructor argument is properly stored / reflected in
// the result of ComputeRestoringVector().
TEST(BuoyancyModelInitialization, ReflectsConstructorArguments) {
  const double volume = 0.5;
  const double density = 1000.0;

  // No offsets: pure Archimedes force, no moments.
  const BuoyancyModel model(volume, density,
                            Eigen::Vector3d(0.0, 0.0, 0.0),
                            Eigen::Vector3d(0.0, 0.0, 0.0),
                            Eigen::Vector3d(0.0, 0.0, kGravity));
  const Eigen::Matrix<double, 6, 1> w = model.ComputeRestoringVector();

  const double expectedForce = -density * volume * kGravity;  // 4905 N up
  EXPECT_NEAR(w(0), 0.0, kTolerance);
  EXPECT_NEAR(w(1), 0.0, kTolerance);
  EXPECT_NEAR(w(2), expectedForce, kTolerance);
  EXPECT_NEAR(w(3), 0.0, kTolerance);
  EXPECT_NEAR(w(4), 0.0, kTolerance);
  EXPECT_NEAR(w(5), 0.0, kTolerance);

  // The method is const: repeated calls return the same value.
  EXPECT_TRUE(w.isApprox(model.ComputeRestoringVector()));

  // Volume and density scale the result linearly (the mass is derived from
  // them, so both the force and the moments follow).
  const BuoyancyModel doubledVolume(2.0 * volume, density,
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, kGravity));
  EXPECT_NEAR(doubledVolume.ComputeRestoringVector()(2), 2.0 * expectedForce,
              kTolerance);

  const BuoyancyModel doubledDensity(volume, 2.0 * density,
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, kGravity));
  EXPECT_NEAR(doubledDensity.ComputeRestoringVector()(2), 2.0 * expectedForce,
              kTolerance);

  // The gravity vector defines the force direction.
  const BuoyancyModel weakerGravity(volume, density,
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, -9.8));
  EXPECT_NEAR(weakerGravity.ComputeRestoringVector()(2), density * volume * 9.8,
              kTolerance);

  // Lateral buoyancy-centre offset produces a moment r_b x F_b.
  const BuoyancyModel offsetBuoyancy(volume, density,
      Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, kGravity));
  const Eigen::Matrix<double, 6, 1> wb = offsetBuoyancy.ComputeRestoringVector();
  EXPECT_NEAR(wb(0), 0.0, kTolerance);
  EXPECT_NEAR(wb(1), 0.0, kTolerance);
  EXPECT_NEAR(wb(2), expectedForce, kTolerance);
  EXPECT_NEAR(wb(3), 0.0, kTolerance);
  EXPECT_NEAR(wb(4), -0.1 * expectedForce, kTolerance);
  EXPECT_NEAR(wb(5), 0.0, kTolerance);

  // The same offset applied to the mass centre gives the opposite moment.
  const BuoyancyModel offsetMass(volume, density,
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.1, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, kGravity));
  const Eigen::Matrix<double, 6, 1> wm = offsetMass.ComputeRestoringVector();
  EXPECT_NEAR(wm(4), 0.1 * expectedForce, kTolerance);

  // Zero volume gives a zero wrench (the derived mass is zero as well).
  const BuoyancyModel empty(0.0, density,
                            Eigen::Vector3d(0.1, 0.0, 0.0),
                            Eigen::Vector3d(0.2, 0.0, 0.0),
                            Eigen::Vector3d(0.0, 0.0, kGravity));
  EXPECT_TRUE(empty.ComputeRestoringVector().isZero(kTolerance));
}

// With the derived (neutral) mass the gravity moment is exactly -r_g x F_b,
// so the total moment equals (r_b - r_g) x F_b.
TEST(BuoyancyModelInitialization, DerivesMassFromVolumeAndDensity) {
  const double volume = 0.5;
  const double density = 1000.0;
  const Eigen::Vector3d buoyancyCenter(0.1, 0.0, 0.0);
  const Eigen::Vector3d massCenter(0.04, 0.0, 0.0);
  const Eigen::Vector3d gravity(0.0, 0.0, kGravity);

  const BuoyancyModel model(volume, density,
                            Eigen::Vector3d(buoyancyCenter),
                            Eigen::Vector3d(massCenter),
                            Eigen::Vector3d(gravity));
  const Eigen::Matrix<double, 6, 1> w = model.ComputeRestoringVector();

  // F_b = (0, 0, 4905); M = (r_b - r_g) x F_b = (0, -0.06 * 4905, 0)
  EXPECT_NEAR(w(2), 4905.0, kTolerance);
  EXPECT_NEAR(w(4), -0.06 * 4905.0, kTolerance);

  // Doubling the volume doubles both the force and the moment.
  const BuoyancyModel bigger(2.0 * volume, density,
      Eigen::Vector3d(buoyancyCenter), Eigen::Vector3d(massCenter),
      Eigen::Vector3d(gravity));
  EXPECT_NEAR(bigger.ComputeRestoringVector()(4), 2.0 * w(4), kTolerance);
}

// Explicit analytic check of the returned [force; moment] vector.
TEST(BuoyancyModelComputation, MatchesAnalyticVector) {
  const double volume = 0.2;
  const double density = 1028.0;
  const double displacedMass = density * volume;  // 205.6 kg (neutral body)
  const Eigen::Vector3d buoyancyCenter(0.0, 0.0, 0.3);
  const Eigen::Vector3d massCenter(0.1, 0.0, 0.0);
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  const BuoyancyModel model(volume, density,
                            Eigen::Vector3d(buoyancyCenter),
                            Eigen::Vector3d(massCenter),
                            Eigen::Vector3d(gravity));
  const Eigen::Matrix<double, 6, 1> w = model.ComputeRestoringVector();

  const double forceZ = displacedMass * 9.81;

  // F_b is parallel to the buoyancy-centre offset -> no moment from F_b;
  // the weight acts at an x-offset -> moment about y.
  const double momentY = massCenter.x() * displacedMass * 9.81;

  EXPECT_NEAR(w(0), 0.0, kTolerance);
  EXPECT_NEAR(w(1), 0.0, kTolerance);
  EXPECT_NEAR(w(2), forceZ, kTolerance);
  EXPECT_NEAR(w(3), 0.0, kTolerance);
  EXPECT_NEAR(w(4), momentY, kTolerance);
  EXPECT_NEAR(w(5), 0.0, kTolerance);

  // Analytically: F_b = (0, 0, 2016.936) N, moment = (0, 201.6936, 0) N*m
  EXPECT_NEAR(w(2), 2016.936, 1e-3);
  EXPECT_NEAR(w(4), 201.6936, 1e-3);
}

// The interface allows substituting mock implementations.
class MockBuoyancyModel : public IBuoyancyModel {
public:
  explicit MockBuoyancyModel(Eigen::Matrix<double, 6, 1> value)
      : m_value(std::move(value)) {}
  Eigen::Matrix<double, 6, 1> ComputeRestoringVector() const override {
    return m_value;
  }
private:
  const Eigen::Matrix<double, 6, 1> m_value;
};

TEST(BuoyancyModelInterface, SupportsMockImplementation) {
  Eigen::Matrix<double, 6, 1> expected;
  expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const std::unique_ptr<IBuoyancyModel> mock =
      std::make_unique<MockBuoyancyModel>(expected);
  EXPECT_TRUE(mock->ComputeRestoringVector().isApprox(expected));
}

}  // namespace
