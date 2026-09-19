#include <IPropellerDynamics.h>
#include <ZeroOrderPropeller.h>

#include <gtest/gtest.h>

#include <memory>

namespace {

constexpr double kTolerance = 1e-12;

TEST(ZeroOrderPropeller, ReturnsTheCommandImmediately) {
  ZeroOrderPropeller propeller;

  EXPECT_NEAR(propeller.Update(0.0, 0.0), 0.0, kTolerance);
  EXPECT_NEAR(propeller.Update(12.5, 1.0), 12.5, kTolerance);
  EXPECT_NEAR(propeller.Update(-3.75, 2.0), -3.75, kTolerance);
  EXPECT_NEAR(propeller.Update(100.0, 50.0), 100.0, kTolerance);
}

TEST(ZeroOrderPropeller, IsUsableThroughTheInterface) {
  const std::unique_ptr<IPropellerDynamics> propeller =
      std::make_unique<ZeroOrderPropeller>();
  EXPECT_NEAR(propeller->Update(42.0, 3.14), 42.0, kTolerance);
}

}  // namespace
