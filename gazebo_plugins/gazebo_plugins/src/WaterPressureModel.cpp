#include <WaterPressureModel.h>

WaterPressureModel::WaterPressureModel(const double fluidDensity,
                                       const double surfaceZ,
                                       const double noiseStddev,
                                       const unsigned int seed)
    : m_fluidDensity(fluidDensity),
      m_surfaceZ(surfaceZ),
      m_noiseStddev(noiseStddev),
      m_generator(seed) {
  if (m_noiseStddev > 0.0)
    m_noise.emplace(0.0, m_noiseStddev);
}

double WaterPressureModel::ComputePressure(const double heightZ) {
  double pressure = kAtmosphericPressure;
  if (heightZ < m_surfaceZ)
    pressure += m_fluidDensity * kGravity * (m_surfaceZ - heightZ);

  if (m_noise)
    pressure += (*m_noise)(m_generator);

  return pressure;
}
