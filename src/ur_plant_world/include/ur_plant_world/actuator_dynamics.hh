// Copyright 2026 Qingyu Geng
//
// Actuator dynamics plugin for the UR plant (Stage 2 of the digital-twin
// plant). Port of the Octave reference model (octave/actuator/):
// per-joint PMSM electrical stage (d-q) + FOC current loop + two-mass
// flexspline, integrated at 10 kHz sub-steps inside the 1 kHz sim loop.
// DART owns the LINK side (inertia, gravity, contacts); this plugin owns
// the motor side (rotor, flexspline, friction) and applies the resolved
// joint torque by overwriting JointForceCmd in PreUpdate.
//
// Model-attached and declared AFTER gz_ros2_control in the URDF, so this
// plugin's PreUpdate runs after the controller wrote the command.
#ifndef UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_
#define UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_

#include <array>
#include <string>
#include <vector>

#include <gz/sim/System.hh>

namespace ur_plant_world
{
/// Per-joint actuator parameters + state (the Octave reference, ported).
struct JointActuator
{
  // --- parameters (plant_config.yaml) ---
  double p{0}, R{0}, L{0}, Kt{0}, lambda{0}, N{101}, Jm{0};
  double k0{0}, k2{0}, d{0}, pC{0}, pv{0}, pvM{0}, pCM{0}, eps{1e-3};
  double Rhat{0}, Lhat{0}, Kthat{0}, lambdahat{0};   // controller belief
  double kp{0}, ki{0};                                // current-loop gains
  // --- state (motor side; the link side lives in DART) ---
  double id{0}, iq{0};         // phase currents, peak
  double thM{0}, wM{0};        // rotor angle / velocity
  double intD{0}, intQ{0};     // PI integrators
  double vdPrev{0}, vqPrev{0}; // one-sample computation delay
  bool rotorInit{false};
  // --- observed (published in 2.3, logged here) ---
  double tauEm{0}, tauFs{0};
  // --- linkage ---
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  std::string name;
};

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
  /// Load the per-joint actuator table from plant_config.yaml
  /// (ur_plant_description share dir, or the <config_file> override).
  bool LoadConfig(const std::string &_path);

  /// Advance one joint by one 100 us sub-step (foc + RK4). DART's joint
  /// state (thJ, wJ) is held constant over the PreUpdate's sub-steps.
  void StepJoint(JointActuator &_j, double _tauJCmd, double _thJ, double _wJ);

  /// One RK4 step of the electrical + rotor ODE (l2).
  static std::array<double, 4> RK4Electrical(const JointActuator &_j,
      const std::array<double, 4> &_x, double _vd, double _vq,
      double _thJ, double _wJ, double _dt);

  /// One RK4 step of the rotor + flexspline ODE only (l1).
  static std::array<double, 2> RK4Mechanical(const JointActuator &_j,
      const std::array<double, 2> &_x, double _tauEm,
      double _thJ, double _wJ, double _dt);

  std::vector<JointActuator> joints_;
  std::string actuatorModel_{"l2"};
  bool sdfModelSet_{false};   // SDF/URDF <actuator_model> wins over the yaml
  double Vdc_{48.0};
  double wc_{2.0 * 3.14159265358979323846 * 500.0};  // rad/s (2*pi*500)
  double dt_{1e-4};      // sub-step = 1/fsw
  int substeps_{10};     // 1 kHz world / 10 kHz internal
  double logPeriod_{1.0};
  double lastLogSimTime_{-1.0};
};
}  // namespace ur_plant_world

#endif  // UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_
