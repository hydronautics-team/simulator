#pragma once

#include <map>
#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <IThrusterConverter.h>

// Factory for thruster converters. The concrete converter is selected by the
// <type> tag of the SDF element passed to Create(), e.g.
//
//   <conversion>
//     <type>Basic</type>
//     <rotorConstant>0.00081</rotorConstant>
//   </conversion>
//
// New converters register themselves with REGISTER_THRUSTER_CONVERTER.
class ThrusterConverterFactory {
public:
  using Creator =
      std::unique_ptr<IThrusterConverter> (*)(const sdf::Element &);

  static ThrusterConverterFactory &GetInstance();

  bool RegisterCreator(const std::string &_identifier, Creator _creator);

  // Returns nullptr if the type tag is missing / unknown or if the converter
  // parameters are invalid.
  std::unique_ptr<IThrusterConverter> Create(const sdf::Element &_sdf) const;

private:
  ThrusterConverterFactory() = default;

  std::map<std::string, Creator> m_creators;
};

// Registers a converter class with the factory. The class must provide a static
//   std::unique_ptr<IThrusterConverter> Create(const sdf::Element &)
//   static const std::string IDENTIFIER
//   static const bool registeredWithFactory
#define REGISTER_THRUSTER_CONVERTER(type, creator)                        \
  const bool type::registeredWithFactory =                                \
      ThrusterConverterFactory::GetInstance().RegisterCreator(           \
          type::IDENTIFIER, creator);
