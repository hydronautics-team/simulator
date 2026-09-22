#pragma once

#include <optional>
#include <random>

// Hydrostatic pressure model of a water column: the physics of a water
// pressure / depth sensor.
//
// All parameters are provided once through the constructor and never change
// afterwards.
//
// Physics (z is the world frame height of the sensor, z up):
//
//   z >= surface_z:  P = P_atm                     (at or above the surface)
//   z <  surface_z:  P = P_atm + rho * g * depth   (depth = surface_z - z)
//
// with g = 9.80665 m/s^2. The optional gaussian noise is added to the
// resulting pressure; with noise_stddev == 0 the model is deterministic and
// repeated calls for the same height return exactly the same value.
//
// The pressure of this model is intended for the water column, not the
// atmosphere: unlike the built-in gz-sensors air_pressure sensor (which uses
// the ISA barometric model and therefore changes by ~12 Pa per metre), the
// gradient here is rho * g (~10 kPa per metre), as on a real depth sensor.
class WaterPressureModel {
public:
  WaterPressureModel(const double fluidDensity,
                     const double surfaceZ = 0.0,
                     const double noiseStddev = 0.0,
                     const unsigned int seed = 0);
  ~WaterPressureModel() = default;

  // Pressure in Pa at the given world frame height.
  double ComputePressure(const double heightZ);

  // Atmospheric pressure at the surface, Pa.
  static constexpr double kAtmosphericPressure = 101325.0;
  // Standard gravity, m/s^2.
  static constexpr double kGravity = 9.80665;

private:
  const double m_fluidDensity;
  const double m_surfaceZ;
  const double m_noiseStddev;
  std::mt19937 m_generator;
  // Only engaged when noise_stddev > 0: a normal distribution with a zero
  // standard deviation is invalid.
  std::optional<std::normal_distribution<double>> m_noise;
};
