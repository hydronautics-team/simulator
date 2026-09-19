#include <PropellerDynamicsFactory.h>

#include <iostream>

PropellerDynamicsFactory &PropellerDynamicsFactory::GetInstance() {
  static PropellerDynamicsFactory instance;
  return instance;
}

bool PropellerDynamicsFactory::RegisterCreator(const std::string &_identifier,
                                               Creator _creator) {
  if (m_creators.find(_identifier) != m_creators.end()) {
    std::cerr << "[PropellerDynamicsFactory] model '" << _identifier
              << "' is already registered" << std::endl;
    return false;
  }
  m_creators[_identifier] = _creator;
  return true;
}

std::unique_ptr<IPropellerDynamics> PropellerDynamicsFactory::Create(
    const sdf::Element &_sdf) const {
  if (!_sdf.HasElement("type")) {
    std::cerr << "[PropellerDynamicsFactory] 'type' element is missing"
              << std::endl;
    return nullptr;
  }

  const std::string identifier = _sdf.Get<std::string>("type");
  const auto it = m_creators.find(identifier);
  if (it == m_creators.end()) {
    std::cerr << "[PropellerDynamicsFactory] unknown type: " << identifier
              << std::endl;
    return nullptr;
  }
  return it->second(_sdf);
}
