// Copyright 2026 Qingyu Geng
//
// Sub-stage 2.1 framework: loads, resolves the robot model's joints, and
// observes the gz_ros2_control effort commands without modifying them.
#include "ur_plant_world/actuator_dynamics.hh"

#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>

namespace ur_plant_world
{
void ActuatorDynamics::Configure(const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  if (_sdf->HasElement("actuator_model"))
    actuatorModel_ = _sdf->Get<std::string>("actuator_model");
  if (_sdf->HasElement("model_name"))
    modelName_ = _sdf->Get<std::string>("model_name");
  if (_sdf->HasElement("log_period"))
    logPeriod_ = _sdf->Get<double>("log_period");

  gzmsg << "ur_plant_world::ActuatorDynamics configured: model='"
        << modelName_ << "', actuator_model=" << actuatorModel_ << "\n";

  // The robot is spawned (ros_gz_sim create) after world Configure, so the
  // model may not exist yet; PreUpdate retries until it does.
  modelResolved_ = ResolveModel(_ecm);
}

bool ActuatorDynamics::ResolveModel(gz::sim::EntityComponentManager &_ecm)
{
  auto modelEntities = _ecm.EntitiesByComponents(
      gz::sim::components::Model(), gz::sim::components::Name(modelName_));
  if (modelEntities.empty())
    return false;

  const gz::sim::Entity modelEntity = *modelEntities.begin();
  auto jointEntities = _ecm.ChildrenByComponents(
      modelEntity, gz::sim::components::Joint());
  joints_.clear();
  jointNames_.clear();
  for (const gz::sim::Entity &jointEntity : jointEntities)
  {
    // Skip fixed joints (e.g. the world -> base_link joint); the actuator
    // acts on the six revolute arm joints only.
    const auto *jointType =
        _ecm.Component<gz::sim::components::JointType>(jointEntity);
    if (jointType && jointType->Data() == sdf::JointType::FIXED)
      continue;
    auto nameComp = _ecm.Component<gz::sim::components::Name>(jointEntity);
    if (nameComp)
    {
      joints_.push_back(jointEntity);
      jointNames_.push_back(nameComp->Data());
    }
  }

  gzmsg << "ur_plant_world::ActuatorDynamics resolved joints: ";
  for (std::size_t i = 0; i < jointNames_.size(); ++i)
    gzmsg << (i ? ", " : "[") << jointNames_[i];
  gzmsg << "]\n";
  return true;
}

void ActuatorDynamics::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!modelResolved_)
  {
    modelResolved_ = ResolveModel(_ecm);
    if (!modelResolved_)
      return;
  }

  if (actuatorModel_ != "l0")
  {
    gzerr << "ur_plant_world::ActuatorDynamics: actuator_model='"
          << actuatorModel_ << "' not implemented yet (2.1 is l0 only)\n";
    return;
  }

  // l0: passthrough — the physics system consumes JointForceCmd unchanged.
  // We only observe it here; 2.2 will intercept and replace it.
  const double simTime =
      std::chrono::duration<double>(_info.simTime).count();
  if (simTime - lastLogSimTime_ < logPeriod_)
    return;
  lastLogSimTime_ = simTime;

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    double cmd = 0.0;
    const auto *forceCmd =
        _ecm.Component<gz::sim::components::JointForceCmd>(joints_[i]);
    if (forceCmd && !forceCmd->Data().empty())
      cmd = forceCmd->Data()[0];
    gzdbg << "actuator l0 passthrough: joint '" << jointNames_[i]
          << "' effort cmd = " << cmd << " Nm\n";
  }
}
}  // namespace ur_plant_world

GZ_ADD_PLUGIN(
    ur_plant_world::ActuatorDynamics,
    gz::sim::System,
    ur_plant_world::ActuatorDynamics::ISystemConfigure,
    ur_plant_world::ActuatorDynamics::ISystemPreUpdate)
