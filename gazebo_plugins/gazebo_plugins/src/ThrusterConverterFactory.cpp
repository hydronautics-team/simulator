#include <ThrusterConverterFactory.h>

#include <iostream>

ThrusterConverterFactory &ThrusterConverterFactory::GetInstance() {
  static ThrusterConverterFactory instance;
  return instance;
}

bool ThrusterConverterFactory::RegisterCreator(const std::string &_identifier,
                                               Creator _creator) {
  if (m_creators.find(_identifier) != m_creators.end()) {
    std::cerr << "[ThrusterConverterFactory] converter '" << _identifier
              << "' is already registered" << std::endl;
    return false;
  }
  m_creators[_identifier] = _creator;
  return true;
}

std::unique_ptr<IThrusterConverter> ThrusterConverterFactory::Create(
    const sdf::Element &_sdf) const {
  if (!_sdf.HasElement("type")) {
    std::cerr << "[ThrusterConverterFactory] 'type' element is missing"
              << std::endl;
    return nullptr;
  }

  const std::string identifier = _sdf.Get<std::string>("type");
  const auto it = m_creators.find(identifier);
  if (it == m_creators.end()) {
    std::cerr << "[ThrusterConverterFactory] unknown type: " << identifier
              << std::endl;
    return nullptr;
  }
  return it->second(_sdf);
}
