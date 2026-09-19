#include <BasicThrusterConverter.h>
#include <IThrusterConverter.h>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

constexpr double kTolerance = 1e-12;

TEST(BasicThrusterConverter, IsQuadraticInPropellerSpeed) {
  const BasicThrusterConverter converter(0.5);

  // Zero command -> zero thrust
  EXPECT_NEAR(converter.Convert(0.0), 0.0, kTolerance);

  // Forward: C * |w| * w = C * w^2
  EXPECT_NEAR(converter.Convert(2.0), 0.5 * 4.0, kTolerance);
  EXPECT_NEAR(converter.Convert(10.0), 0.5 * 100.0, kTolerance);

  // Reversed propeller -> reversed thrust
  EXPECT_NEAR(converter.Convert(-2.0), -0.5 * 4.0, kTolerance);
  EXPECT_NEAR(converter.Convert(-10.0), -0.5 * 100.0, kTolerance);

  // The curve is symmetric in magnitude
  EXPECT_NEAR(converter.Convert(-3.0), -converter.Convert(3.0), kTolerance);
}

TEST(BasicThrusterConverter, ScalesWithTheCoefficient) {
  const BasicThrusterConverter weak(0.1);
  const BasicThrusterConverter strong(0.4);

  EXPECT_NEAR(strong.Convert(5.0), 4.0 * weak.Convert(5.0), kTolerance);
}

TEST(BasicThrusterConverter, IsUsableThroughTheInterface) {
  const std::unique_ptr<IThrusterConverter> converter =
      std::make_unique<BasicThrusterConverter>(0.25);
  EXPECT_NEAR(converter->Convert(4.0), 4.0, kTolerance);
}

}  // namespace
