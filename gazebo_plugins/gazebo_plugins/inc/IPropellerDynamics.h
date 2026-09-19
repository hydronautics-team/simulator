#pragma once

// Interface for the dynamics of a propeller (thruster rotor).
//
// Mirrors the legacy `Dynamics` class from uuv_gazebo_plugins
// (see legacy_classic / uuv_gazebo_plugins): `Update(command, time)` returns
// the current propeller state (e.g. the rotor angular velocity) for a command
// value at a given time.
//
// The method is not const because real dynamics implementations keep internal
// state (previous time / previous state), e.g. first-order dynamics with a
// time constant.
class IPropellerDynamics {
public:
  virtual ~IPropellerDynamics() = default;

  // \param[in] _command desired propeller state (e.g. rotor speed command)
  // \param[in] _timeSeconds current simulation time
  // \return the propeller state after applying the dynamics
  virtual double Update(double _command, double _timeSeconds) = 0;
};
