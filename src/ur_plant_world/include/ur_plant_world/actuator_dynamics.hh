// Copyright 2026 Qingyu Geng
//
// Actuator dynamics plugin for the UR plant (Stage 2 of the digital-twin
// plant). Sub-stage 2.1: framework only — reads the effort command that
// gz_ros2_control writes on each joint (components::JointForceCmd) and
// passes it through unchanged (actuator_model = l0). The L2 PMSM/two-mass
// cascade replaces this passthrough in 2.2/2.3.
#ifndef UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_
#define UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_

#include <string>
#include <vector>

#include <gz/sim/System.hh>

namespace ur_plant_world
{
class ActuatorDynamics
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  ActuatorDynamics() = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  /// Find the robot model by name and fill joints_/jointNames_.
  /// Returns true when the model was found (it is spawned after world
  /// Configure, so PreUpdate retries until it appears).
  bool ResolveModel(gz::sim::EntityComponentManager &_ecm);

  /// Joint entities of the robot model, in config order.
  std::vector<gz::sim::Entity> joints_;
  /// Joint names (logging now; topic names in 2.3).
  std::vector<std::string> jointNames_;
  /// Model resolution state (the model is spawned after Configure).
  bool modelResolved_{false};
  /// Actuator model selector: l0 passthrough for now, l1/l2 in 2.2/2.3.
  std::string actuatorModel_{"l0"};
  /// Name of the robot model the plugin acts on.
  std::string modelName_{"ur"};
  /// Seconds between per-joint command logs.
  double logPeriod_{1.0};
  /// Sim time (s) of the last command log.
  double lastLogSimTime_{-1.0};
};
}  // namespace ur_plant_world

#endif  // UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_
