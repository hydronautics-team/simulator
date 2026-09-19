#pragma once

#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <IThrusterConverter.h>

// Simplest thruster converter (legacy equivalent: ConversionFunctionBasic):
//   thrust = coefficient * |omega| * omega
// The sign of the thrust follows the sign of the propeller speed, so a
// reversed propeller produces reversed thrust.
//
// SDF:
//   <conversion>
//     <type>Basic</type>
//     <rotorConstant>0.00081</rotorConstant>
//   </conversion>
class BasicThrusterConverter : public IThrusterConverter {
public:
  explicit BasicThrusterConverter(double _coefficient);
  ~BasicThrusterConverter() override = default;

  double Convert(double _propellerSpeed) const override;

  // Factory entry point: reads <rotorConstant> (legacy tag name). Returns
  // nullptr if the parameter is missing.
  static std::unique_ptr<IThrusterConverter> Create(const sdf::Element &_sdf);

  static const std::string IDENTIFIER;

private:
  static const bool registeredWithFactory;

  const double m_coefficient;
};
