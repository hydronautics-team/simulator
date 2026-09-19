#include <BasicThrusterConverter.h>

#include <ThrusterConverterFactory.h>

#include <cmath>
#include <iostream>

const std::string BasicThrusterConverter::IDENTIFIER = "Basic";
REGISTER_THRUSTER_CONVERTER(BasicThrusterConverter,
                            &BasicThrusterConverter::Create)

std::unique_ptr<IThrusterConverter> BasicThrusterConverter::Create(
    const sdf::Element &_sdf) {
  if (!_sdf.HasElement("rotorConstant")) {
    std::cerr << "[BasicThrusterConverter] expected element 'rotorConstant'"
              << std::endl;
    return nullptr;
  }
  return std::make_unique<BasicThrusterConverter>(
      _sdf.Get<double>("rotorConstant"));
}

BasicThrusterConverter::BasicThrusterConverter(double _coefficient)
    : m_coefficient(_coefficient) {}

double BasicThrusterConverter::Convert(double _propellerSpeed) const {
  return m_coefficient * std::fabs(_propellerSpeed) * _propellerSpeed;
}
