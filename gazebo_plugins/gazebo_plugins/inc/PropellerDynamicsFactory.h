#pragma once

#include <map>
#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <IPropellerDynamics.h>

// Factory for propeller dynamics models. The concrete model is selected by the
// <type> tag of the SDF element passed to Create(), e.g.
//
//   <dynamics>
//     <type>ZeroOrder</type>
//   </dynamics>
//
// New models register themselves with REGISTER_PROPELLER_DYNAMICS.
class PropellerDynamicsFactory {
public:
  using Creator =
      std::unique_ptr<IPropellerDynamics> (*)(const sdf::Element &);

  static PropellerDynamicsFactory &GetInstance();

  bool RegisterCreator(const std::string &_identifier, Creator _creator);

  // Returns nullptr if the type tag is missing / unknown or if the model
  // parameters are invalid.
  std::unique_ptr<IPropellerDynamics> Create(const sdf::Element &_sdf) const;

private:
  PropellerDynamicsFactory() = default;

  std::map<std::string, Creator> m_creators;
};

// Registers a model class with the factory. The class must provide a static
//   std::unique_ptr<IPropellerDynamics> Create(const sdf::Element &)
//   static const std::string IDENTIFIER
//   static const bool registeredWithFactory
#define REGISTER_PROPELLER_DYNAMICS(type, creator)                        \
  const bool type::registeredWithFactory =                                \
      PropellerDynamicsFactory::GetInstance().RegisterCreator(            \
          type::IDENTIFIER, creator);
