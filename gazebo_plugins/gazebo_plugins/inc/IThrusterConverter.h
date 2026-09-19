#pragma once

// Interface for a thruster converter: maps the propeller state (e.g. the rotor
// angular velocity) to the thrust force produced by the thruster.
//
// Legacy equivalent: the `ConversionFunction` hierarchy from
// uuv_gazebo_plugins (ThrusterConversionFcn.h), e.g. `Basic` with
//   thrust = rotorConstant * |omega| * omega
class IThrusterConverter {
public:
  virtual ~IThrusterConverter() = default;

  // \param[in] _propellerSpeed propeller state (e.g. rotor angular velocity)
  // \return thrust force produced by the propeller
  virtual double Convert(double _propellerSpeed) const = 0;
};
