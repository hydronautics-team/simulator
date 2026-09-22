#include <WaterPressureModel.h>

#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-9;
constexpr double kAtmospheric = WaterPressureModel::kAtmosphericPressure;
constexpr double kGravity = WaterPressureModel::kGravity;

// At and above the surface the model reports the atmospheric pressure.
TEST(WaterPressureModel, SurfaceAndAbove) {
  WaterPressureModel model(1028.0);

  EXPECT_NEAR(model.ComputePressure(0.0), kAtmospheric, kTolerance);
  EXPECT_NEAR(model.ComputePressure(5.0), kAtmospheric, kTolerance);
}

// Below the surface the excess pressure is rho * g * depth.
TEST(WaterPressureModel, HydrostaticPressureAtDepth) {
  const double density = 1028.0;
  WaterPressureModel model(density);

  const double depth = 20.0;
  const double expected = kAtmospheric + density * kGravity * depth;
  EXPECT_NEAR(model.ComputePressure(-depth), expected, 1e-6);
}

// The surface height shifts the water column: a sensor in a tank whose
// surface is at z = 5 m reads the atmospheric pressure there and the water
// pressure below it.
TEST(WaterPressureModel, CustomSurfaceHeight) {
  const double density = 1000.0;
  const double surfaceZ = 5.0;
  WaterPressureModel model(density, surfaceZ);

  EXPECT_NEAR(model.ComputePressure(surfaceZ), kAtmospheric, kTolerance);
  EXPECT_NEAR(model.ComputePressure(0.0),
              kAtmospheric + density * kGravity * surfaceZ, 1e-6);
}

// The excess pressure scales linearly with the depth and with the density.
TEST(WaterPressureModel, ScalesWithDepthAndDensity) {
  WaterPressureModel model(1028.0);
  const double p10 = model.ComputePressure(-10.0) - kAtmospheric;
  const double p20 = model.ComputePressure(-20.0) - kAtmospheric;
  EXPECT_NEAR(p20, 2.0 * p10, 1e-6);

  WaterPressureModel doubledDensity(2056.0);
  EXPECT_NEAR(doubledDensity.ComputePressure(-10.0) - kAtmospheric,
              2.0 * p10, 1e-6);
}

// Without noise the model is deterministic: repeated calls for the same
// height return exactly the same pressure.
TEST(WaterPressureModel, NoNoiseIsDeterministic) {
  WaterPressureModel model(1028.0);
  EXPECT_DOUBLE_EQ(model.ComputePressure(-5.0),
                   model.ComputePressure(-5.0));
}

// With noise enabled the samples scatter around the analytic value (a rough
// bound: 100 draws must not be all identical).
TEST(WaterPressureModel, NoiseProducesScatteredSamples) {
  WaterPressureModel model(1028.0, 0.0, 100.0, 42u);
  const double first = model.ComputePressure(-10.0);

  bool foundDifferent = false;
  for (int i = 0; i < 100 && !foundDifferent; ++i)
    foundDifferent = model.ComputePressure(-10.0) != first;

  EXPECT_TRUE(foundDifferent);
}

}  // namespace
