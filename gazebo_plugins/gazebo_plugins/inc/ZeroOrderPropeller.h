#pragma once

#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <IPropellerDynamics.h>

// Simplest propeller dynamics: no dynamics at all, the command is returned
// as the propeller state (legacy equivalent: DynamicsZeroOrder).
//
// SDF:
//   <dynamics>
//     <type>ZeroOrder</type>
//   </dynamics>
class ZeroOrderPropeller : public IPropellerDynamics {
public:
  ZeroOrderPropeller() = default;
  ~ZeroOrderPropeller() override = default;

  double Update(double _command, double _timeSeconds) override;

  // Factory entry point: the model carries no parameters, so the SDF element
  // is accepted (and ignored) only to keep a uniform creator signature.
  static std::unique_ptr<IPropellerDynamics> Create(const sdf::Element &_sdf);

  static const std::string IDENTIFIER;

private:
  static const bool registeredWithFactory;
};
