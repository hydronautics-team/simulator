#include <ZeroOrderPropeller.h>

#include <PropellerDynamicsFactory.h>

const std::string ZeroOrderPropeller::IDENTIFIER = "ZeroOrder";
REGISTER_PROPELLER_DYNAMICS(ZeroOrderPropeller, &ZeroOrderPropeller::Create)

std::unique_ptr<IPropellerDynamics> ZeroOrderPropeller::Create(
    const sdf::Element &_sdf) {
  // No parameters to read: the propeller state follows the command directly.
  (void)_sdf;
  return std::make_unique<ZeroOrderPropeller>();
}

double ZeroOrderPropeller::Update(double _command, double _timeSeconds) {
  // No dynamics: the propeller state follows the command immediately.
  (void)_timeSeconds;
  return _command;
}
