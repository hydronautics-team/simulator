#include <WaterPressurePlugin.h>

#include <chrono>
#include <string>

#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>

namespace {

// Reads an optional scalar tag, keeping the current value when absent.
double ReadDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                  const std::string &_tag, double _current) {
  if (!_sdf->HasElement(_tag))
    return _current;
  return _sdf->Get<double>(_tag);
}

}  // namespace

WaterPressurePlugin::WaterPressurePlugin() = default;

WaterPressurePlugin::~WaterPressurePlugin() = default;

void WaterPressurePlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) {
  this->m_model = gz::sim::Model(_entity);

  std::string linkName = "base_link";
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");

  const auto linkEntity = this->m_model.LinkByName(_ecm, linkName);
  if (linkEntity == gz::sim::kNullEntity) {
    gzwarn << "[WaterPressurePlugin] link '" << linkName
            << "' not found in model '" << this->m_model.Name(_ecm) << "'"
            << std::endl;
    return;
  }
  this->m_link = gz::sim::Link(linkEntity);
  this->m_linkName = linkName;

  // The pressure is computed from the world pose of the link. Enabling the
  // velocity checks creates the WorldPose component (with the current pose)
  // like in the thruster plugin; a hand-made empty component would hold an
  // identity pose.
  this->m_link.EnableVelocityChecks(_ecm, true);

  const double surfaceZ = ReadDouble(_sdf, "surface_z", 0.0);
  const double fluidDensity = ReadDouble(_sdf, "fluid_density", 1028.0);
  const double noiseStddev = ReadDouble(_sdf, "noise_stddev", 0.0);
  if (_sdf->HasElement("debug"))
    this->m_debugMode = _sdf->Get<bool>("debug");

  const double updateRate = ReadDouble(_sdf, "update_rate", 10.0);
  if (updateRate > 0.0) {
    this->m_updatePeriodSeconds = 1.0 / updateRate;
  } else {
    gzwarn << "[WaterPressurePlugin] 'update_rate' must be positive, using "
               "10 Hz" << std::endl;
  }

  this->m_pressureModel = std::make_unique<WaterPressureModel>(
      fluidDensity, surfaceZ, noiseStddev);

  const std::string modelName = this->m_model.Name(_ecm);
  std::string topic = "/" + modelName + "/sensors/pressure";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->m_node = std::make_shared<gz::transport::Node>();
  this->m_publisher = this->m_node->Advertise<gz::msgs::FluidPressure>(topic);

  gzmsg << "[WaterPressurePlugin] attached to link '" << linkName
         << "' of model '" << modelName << "', surface z " << surfaceZ
         << " m, rho " << fluidDensity << " kg/m^3, " << updateRate
         << " Hz, topic '" << topic << "'" << std::endl;
}

void WaterPressurePlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {
  if (this->m_link.Entity() == gz::sim::kNullEntity || !this->m_pressureModel)
    return;

  if (_info.paused)
    return;

  const double timeSeconds =
      std::chrono::duration<double>(_info.simTime).count();
  if (this->m_hasPublished &&
      timeSeconds - this->m_lastPublishSeconds < this->m_updatePeriodSeconds)
    return;

  const auto pose = this->m_link.WorldPose(_ecm);
  if (!pose)
    return;

  const double pressure =
      this->m_pressureModel->ComputePressure(pose->Pos().Z());

  gz::msgs::FluidPressure message;
  *message.mutable_header()->mutable_stamp() = gz::msgs::Convert(_info.simTime);
  auto *frame = message.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->m_model.Name(_ecm) + "::" + this->m_linkName);
  message.set_pressure(pressure);
  this->m_publisher.Publish(message);

  this->m_lastPublishSeconds = timeSeconds;
  this->m_hasPublished = true;

  if (this->m_debugMode && ++this->m_debugCounter % 100 == 0)
    gzmsg << "[WaterPressurePlugin][debug] t=" << _info.simTime.count()
           << " ns, z=" << pose->Pos().Z() << " m, p=" << pressure << " Pa"
           << std::endl;
}

GZ_ADD_PLUGIN(WaterPressurePlugin, gz::sim::System,
              WaterPressurePlugin::ISystemConfigure,
              WaterPressurePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WaterPressurePlugin, "water_pressure")
