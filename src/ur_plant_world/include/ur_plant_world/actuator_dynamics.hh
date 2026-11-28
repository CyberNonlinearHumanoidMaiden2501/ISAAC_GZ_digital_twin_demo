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
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace ur_plant_world
{
/// The one DC bus shared by all six joint drives (octave inverter.m).
struct BusState
{
  double Vdc{48.0}, Vsrc{48.0}, Rsrc{0.05}, C{0.01};
  double tDt{2e-6}, epsS{0.5}, Vclamp{55.0};
};

/// Current-measurement model (octave sense_current.m).
struct SenseConfig
{
  double iRange{30.0}, bias{0.02};
  bool rippleOn{false};
};

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
  // --- observed / published ---
  double tauEm{0}, tauFs{0}, tauJ{0};
  double idMeasured{0}, iqMeasured{0};
  // --- direct effort override (gz topic; NaN = use the JTC command) ---
  double effortOverride{std::numeric_limits<double>::quiet_NaN()};
  // --- linkage ---
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  std::string name;
  // --- publishers (2.3) ---
  gz::transport::Node::Publisher pubCurrent;
  gz::transport::Node::Publisher pubTorque;
  gz::transport::Node::Publisher pubWindup;
  gz::transport::Node::Publisher pubMotorVelocity;
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

  /// Per-joint effort override (gz topic; routed by topic name).
  void OnEffortOverride(const gz::msgs::Double &_msg,
                        const gz::transport::MessageInfo &_info);

private:
  /// Load the per-joint actuator table from plant_config.yaml
  /// (ur_plant_description share dir, or the <config_file> override).
  bool LoadConfig(const std::string &_path);

  /// Advance one joint by one 100 us sub-step (foc + inverter + RK4).
  /// DART's joint state (thJ, wJ) is held over the PreUpdate's sub-steps.
  /// Returns the electrical power drawn by the joint (for the bus state).
  double StepJoint(JointActuator &_j, double _tauJCmd, double _thJ,
                   double _wJ, BusState &_bus);

  /// One RK4 step of the electrical + rotor ODE (l2).
  static std::array<double, 4> RK4Electrical(const JointActuator &_j,
      const std::array<double, 4> &_x, double _vd, double _vq,
      double _thJ, double _wJ, double _dt);

  /// One RK4 step of the rotor + flexspline ODE only (l1).
  static std::array<double, 2> RK4Mechanical(const JointActuator &_j,
      const std::array<double, 2> &_x, double _tauEm,
      double _thJ, double _wJ, double _dt);

  std::vector<JointActuator> joints_;
  BusState bus_;
  SenseConfig sense_;
  double stictionRatio_{1.3};   // static friction = ratio * Coulomb [E]
  double stictionVs_{0.01};     // Stribeck velocity, rad/s [E]
  std::string actuatorModel_{"l2"};
  bool sdfModelSet_{false};   // SDF/URDF <actuator_model> wins over the yaml
  double wc_{2.0 * 3.14159265358979323846 * 500.0};  // rad/s (2*pi*500)
  double dt_{1e-4};      // sub-step = 1/fsw
  int substeps_{10};     // 1 kHz world / 10 kHz internal
  double logPeriod_{1.0};
  double lastLogSimTime_{-1.0};
  gz::transport::Node node_;
  gz::transport::Node::Publisher pubBusVoltage_;
  std::mt19937 rng_{1234};                 // ripple injection (sense model)
  std::normal_distribution<double> normal_{0.0, 1.0};
};
}  // namespace ur_plant_world

#endif  // UR_PLANT_WORLD__ACTUATOR_DYNAMICS_HH_
