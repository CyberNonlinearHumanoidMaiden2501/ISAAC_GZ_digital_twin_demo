// Copyright 2026 Qingyu Geng
//
// Port of the Octave reference model (octave/actuator/):
//   foc_loop.m      -> StepJoint's PI + decoupling + one-sample delay
//   inverter.m      -> dead-time in d-q + bus state (shared, diode-isolated)
//   pmsm.m          -> RK4Electrical (d-q ODE)
//   two_mass.m      -> rotor side of RK4Electrical / RK4Mechanical
//   sense_current.m -> measurement model (midpoint average + ADC + bias)
//   params*.m       -> plant_config.yaml `actuator:` table
// DART owns the LINK side; the plugin applies the resolved joint torque by
// overwriting JointForceCmd (PreUpdate runs after gz_ros2_control — the
// plugin is declared after it in the URDF).
#include "ur_plant_world/actuator_dynamics.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>
#include <yaml-cpp/yaml.h>

namespace ur_plant_world
{
namespace
{
constexpr char kTopicPrefix[] = "/plant/actuator/";
constexpr double kTwoPiOver3 = 2.0943951023931953;  // 2*pi/3
}  // namespace

void ActuatorDynamics::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  // Model-attached plugin: _entity is the robot model.
  if (_sdf->HasElement("actuator_model"))
  {
    actuatorModel_ = _sdf->Get<std::string>("actuator_model");
    sdfModelSet_ = true;
  }
  if (_sdf->HasElement("log_period"))
    logPeriod_ = _sdf->Get<double>("log_period");

  auto jointEntities = _ecm.ChildrenByComponents(
      _entity, gz::sim::components::Joint());
  for (const gz::sim::Entity &jointEntity : jointEntities)
  {
    const auto *jointType =
        _ecm.Component<gz::sim::components::JointType>(jointEntity);
    if (jointType && jointType->Data() == sdf::JointType::FIXED)
      continue;  // skip the world -> base_link joint
    auto nameComp = _ecm.Component<gz::sim::components::Name>(jointEntity);
    if (!nameComp)
      continue;
    JointActuator j;
    j.jointEntity = jointEntity;
    j.name = nameComp->Data();
    joints_.push_back(std::move(j));
  }

  // Parameter source: plant_config.yaml, resolved via ament unless the SDF
  // block overrides it.
  std::string configPath;
  if (_sdf->HasElement("config_file"))
    configPath = _sdf->Get<std::string>("config_file");
  else
    configPath = ament_index_cpp::get_package_share_directory(
                     "ur_plant_description") +
                 "/config/plant_config.yaml";
  if (!LoadConfig(configPath))
  {
    gzerr << "ur_plant_world::ActuatorDynamics: config load failed ("
          << configPath << "); plugin inert\n";
    joints_.clear();
    return;
  }

  // State publishers + per-joint effort override subscriptions.
  pubBusVoltage_ =
      node_.Advertise<gz::msgs::Double>(std::string(kTopicPrefix) +
                                        "bus_voltage");
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    auto &j = joints_[i];
    const std::string base = std::string(kTopicPrefix) + j.name;
    j.pubCurrent = node_.Advertise<gz::msgs::Vector3d>(base + "/motor_current");
    j.pubTorque = node_.Advertise<gz::msgs::Vector3d>(base + "/torque");
    j.pubWindup = node_.Advertise<gz::msgs::Double>(base + "/windup");
    j.pubMotorVelocity =
        node_.Advertise<gz::msgs::Double>(base + "/motor_velocity");
    node_.Subscribe(base + "/effort_override",
        &ActuatorDynamics::OnEffortOverride, this);
  }

  gzmsg << "ur_plant_world::ActuatorDynamics configured: actuator_model="
        << actuatorModel_ << ", joints=";
  for (std::size_t i = 0; i < joints_.size(); ++i)
    gzmsg << (i ? ", " : "[") << joints_[i].name;
  gzmsg << "]\n";
}

bool ActuatorDynamics::LoadConfig(const std::string &_path)
{
  YAML::Node act;
  try
  {
    act = YAML::LoadFile(_path)["actuator"];
  }
  catch (const YAML::Exception &e)
  {
    gzerr << "ActuatorDynamics: yaml error: " << e.what() << "\n";
    return false;
  }
  if (!act)
  {
    gzerr << "ActuatorDynamics: no 'actuator:' section in " << _path << "\n";
    return false;
  }

  if (act["model"] && !sdfModelSet_)
    actuatorModel_ = act["model"].as<std::string>();
  if (act["fsw"])
    dt_ = 1.0 / act["fsw"].as<double>();
  if (act["wc"])
    wc_ = act["wc"].as<double>();
  const double errR = act["id_err"]["R"].as<double>();
  const double errL = act["id_err"]["L"].as<double>();
  const double errKt = act["id_err"]["Kt"].as<double>();

  if (act["bus"])
  {
    const YAML::Node b = act["bus"];
    bus_.Vdc = b["Vdc"].as<double>();
    bus_.Vsrc = b["Vsrc"].as<double>();
    bus_.Rsrc = b["Rsrc"].as<double>();
    bus_.C = b["C"].as<double>();
    bus_.tDt = b["t_dt"].as<double>();
    bus_.Vclamp = b["V_clamp"].as<double>();
    bus_.epsS = b["eps_s"].as<double>();
  }
  if (act["sense"])
  {
    sense_.iRange = act["sense"]["I_range"].as<double>();
    sense_.bias = act["sense"]["bias"].as<double>();
    sense_.rippleOn = act["sense"]["ripple_on"].as<bool>();
  }
  if (act["stiction"])
  {
    stictionRatio_ = act["stiction"]["ratio"].as<double>();
    stictionVs_ = act["stiction"]["vs"].as<double>();
  }

  for (auto &j : joints_)
  {
    const YAML::Node c = act["joints"][j.name];
    if (!c)
    {
      gzerr << "ActuatorDynamics: joint '" << j.name
            << "' missing in the actuator config\n";
      return false;
    }
    j.p = c["p"].as<double>();
    j.R = c["R"].as<double>();
    j.L = c["L"].as<double>();
    j.Kt = c["Kt"].as<double>();
    j.Jm = c["Jm"].as<double>();
    j.N = c["N"].as<double>();
    j.k0 = c["k0"].as<double>();
    j.d = c["d"].as<double>();
    j.pC = c["pC"].as<double>();
    j.pv = c["pv"].as<double>();
    j.pvM = c["pv_m"].as<double>();
    j.pCM = c["pC_m"].as<double>();
    j.eps = c["eps"].as<double>();

    // Derived values (as in the Octave reference).
    j.lambda = j.Kt / (1.5 * j.p);
    j.k2 = c["s_fac"].as<double>() * j.k0 /
           (c["phi_ref"].as<double>() * c["phi_ref"].as<double>());
    // Controller belief: identification errors on the true parameters.
    j.Rhat = j.R * (1.0 + errR);
    j.Lhat = j.L * (1.0 + errL);
    j.Kthat = j.Kt * (1.0 + errKt);
    j.lambdahat = j.Kthat / (1.5 * j.p);
    j.kp = j.Lhat * wc_;
    j.ki = j.Rhat * wc_;
  }
  return true;
}

void ActuatorDynamics::OnEffortOverride(const gz::msgs::Double &_msg,
    const gz::transport::MessageInfo &_info)
{
  for (auto &j : joints_)
  {
    if (_info.Topic() ==
        std::string(kTopicPrefix) + j.name + "/effort_override")
      j.effortOverride = _msg.data();  // NaN disables the override
  }
}

void ActuatorDynamics::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  const double simTime =
      std::chrono::duration<double>(_info.simTime).count();
  const bool logNow = simTime - lastLogSimTime_ >= logPeriod_;

  if (actuatorModel_ == "l0")
  {
    if (logNow)
    {
      for (const auto &j : joints_)
      {
        double cmd = 0.0;
        const auto *forceCmd =
            _ecm.Component<gz::sim::components::JointForceCmd>(j.jointEntity);
        if (forceCmd && !forceCmd->Data().empty())
          cmd = forceCmd->Data()[0];
        gzdbg << "actuator l0 passthrough: '" << j.name << "' cmd=" << cmd
              << " Nm\n";
      }
      lastLogSimTime_ = simTime;
    }
    return;  // passthrough: the command flows to the physics unchanged
  }

  // Pre-read commands (effort override wins over the JTC) and DART state.
  std::vector<double> cmd(joints_.size()), thJ(joints_.size()),
      wJ(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    auto &j = joints_[i];
    cmd[i] = 0.0;
    const auto *forceCmd =
        _ecm.Component<gz::sim::components::JointForceCmd>(j.jointEntity);
    if (forceCmd && !forceCmd->Data().empty())
      cmd[i] = forceCmd->Data()[0];
    if (!std::isnan(j.effortOverride))
      cmd[i] = j.effortOverride;
    thJ[i] = 0.0;
    wJ[i] = 0.0;
    const auto *pos =
        _ecm.Component<gz::sim::components::JointPosition>(j.jointEntity);
    const auto *vel =
        _ecm.Component<gz::sim::components::JointVelocity>(j.jointEntity);
    if (pos && !pos->Data().empty())
      thJ[i] = pos->Data()[0];
    if (vel && !vel->Data().empty())
      wJ[i] = vel->Data()[0];
    if (!j.rotorInit)
    {
      j.thM = j.N * thJ[i];
      j.wM = j.N * wJ[i];
      j.rotorInit = true;
    }
  }

  // 10 kHz sub-steps: all joints step, then the shared bus integrates.
  for (int s = 0; s < substeps_; ++s)
  {
    double pTotal = 0.0;
    for (std::size_t i = 0; i < joints_.size(); ++i)
      pTotal += StepJoint(joints_[i], cmd[i], thJ[i], wJ[i], bus_);
    const double pSrc =
        std::max(0.0, bus_.Vsrc * (bus_.Vsrc - bus_.Vdc) / bus_.Rsrc);
    bus_.Vdc += (pSrc - pTotal) / bus_.Vdc / bus_.C * dt_;
    bus_.Vdc = std::min(std::max(bus_.Vdc, 0.0), bus_.Vclamp);
  }

    // Resolved torque + measurement, then apply to the joint and publish.
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
    auto &j = joints_[i];
    const double phi = j.thM / j.N - thJ[i];
    const double phiDot = j.wM / j.N - wJ[i];
    // Physical sanity guards: the flexspline is a stiff spring, and an
    // uncontrolled windup transient saturates the electrical stage — the
    // clamped values keep every quantity finite (a NaN torque would
    // poison the physics engine).
    const double phiC = std::clamp(phi, -0.5, 0.5);
    j.tauFs = j.k0 * phiC + j.k2 * phiC * phiC * phiC +
              j.d * std::clamp(phiDot, -50.0, 50.0);
    // Stribeck stiction with a proper stick regime: below the breakaway
    // velocity the friction BALANCES the spring torque up to the static
    // limit (otherwise the light wrist joints free-drift at rest — the
    // smoothed Coulomb alone has no static friction); above it, the
    // classic Stribeck roll-off to the Coulomb level.
    const double pCs = stictionRatio_ * j.pC;
    double fric;
    if (std::abs(wJ[i]) < stictionVs_)
    {
      fric = std::clamp(j.tauFs, -pCs, pCs);
    }
    else
    {
      const double stribeck =
          j.pC + (pCs - j.pC) *
                     std::exp(-(wJ[i] / stictionVs_) * (wJ[i] / stictionVs_));
      fric = stribeck * std::tanh(wJ[i] / j.eps) + j.pv * wJ[i];
    }
      // The reported joint velocity is left to the physics (gz_ros2_control
      // reads the physics API, not this component — a write here could not
      // reach it). The velocity-limit garbage it produced for the light
      // joints is prevented at the source: the stick regime below keeps
      // the joints from drifting into the DART velocity limiter.
      j.tauJ = j.tauFs - fric;

    // Never hand the physics a non-finite state: reset the joint and
    // apply zero torque for this iteration.
    if (!std::isfinite(j.tauJ) || !std::isfinite(j.wM) ||
        !std::isfinite(j.iq))
    {
      gzerr << "actuator: '" << j.name
            << "' non-finite state — resetting the joint\n";
      j.id = 0.0;
      j.iq = 0.0;
      j.intD = 0.0;
      j.intQ = 0.0;
      j.vdPrev = 0.0;
      j.vqPrev = 0.0;
      j.thM = j.N * thJ[i];
      j.wM = j.N * wJ[i];
      j.tauEm = 0.0;
      j.tauFs = 0.0;
      j.tauJ = 0.0;
    }

    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(
        j.jointEntity, std::vector<double>{j.tauJ});

      // Resolved torque is applied below; the joint-velocity state is
      // left to the physics (gz_ros2_control reads the physics API, not
      // the ECM component — a component write could not reach it).

    // Measurement: midpoint average + 12-bit ADC + bias (+ optional ripple).
    const double vMag = std::sqrt(j.vdPrev * j.vdPrev + j.vqPrev * j.vqPrev);
    const double mod =
        std::min(1.0, vMag / (bus_.Vdc / std::sqrt(3.0)));
    double rip = 0.0;
    if (sense_.rippleOn)
    {
      const double pp = bus_.Vdc * dt_ * mod / (8.0 * j.L);
      rip = pp / (2.0 * std::sqrt(3.0)) * normal_(rng_);
    }
    const double lsb = 2.0 * sense_.iRange / 4096.0;
    j.idMeasured = std::round((j.id + rip) / lsb) * lsb + sense_.bias;
    j.iqMeasured = std::round((j.iq + rip) / lsb) * lsb + sense_.bias;

    gz::msgs::Vector3d cMsg;
    cMsg.set_x(j.idMeasured);
    cMsg.set_y(j.iqMeasured);
    j.pubCurrent.Publish(cMsg);
    gz::msgs::Vector3d tMsg;
    tMsg.set_x(j.tauEm);
    tMsg.set_y(j.tauFs);
    tMsg.set_z(j.tauJ);
    j.pubTorque.Publish(tMsg);
    gz::msgs::Double wMsg;
    wMsg.set_data(phi);
    j.pubWindup.Publish(wMsg);
    gz::msgs::Double vMsg;
    vMsg.set_data(j.wM);
    j.pubMotorVelocity.Publish(vMsg);

    if (logNow)
    {
      gzdbg << "actuator: '" << j.name << "' cmd=" << cmd[i]
            << " Nm -> tau_fs=" << j.tauFs << " tau_j=" << j.tauJ
            << " i_q=" << j.iq << " windup=" << phi << " rad\n";
    }
  }
  gz::msgs::Double bMsg;
  bMsg.set_data(bus_.Vdc);
  pubBusVoltage_.Publish(bMsg);
  if (logNow)
    lastLogSimTime_ = simTime;
}

double ActuatorDynamics::StepJoint(JointActuator &j, double _tauJCmd,
    double _thJ, double _wJ, BusState &_bus)
{
  // Motor torque reference: joint effort reflected through the gearbox,
  // plus the motor-side friction feedforward (zero in the current config).
  const double tauMStar =
      _tauJCmd / j.N + (j.pCM * std::tanh(j.wM / j.eps) + j.pvM * j.wM);

  if (actuatorModel_ == "l1")
  {
    // Two-mass without the electrical stage: tau_em = tau_m*.
    j.tauEm = tauMStar;
    std::array<double, 2> x{j.thM, j.wM};
    x = RK4Mechanical(j, x, j.tauEm, _thJ, _wJ, dt_);
    j.thM = x[0];
    j.wM = x[1];
    return 0.0;  // no electrical power in l1
  }

  // FOC current loop: belief gains/decoupling, circular clamp at the
  // actual bus, one-sample computation delay.
  const double we = j.p * j.wM;
  const double iqStar = tauMStar / j.Kthat;
  const double eD = -j.id;
  const double eQ = iqStar - j.iq;
  j.intD += j.ki * eD * dt_;
  j.intQ += j.ki * eQ * dt_;
  double vdRaw = j.kp * eD + j.intD - we * j.Lhat * j.iq;
  double vqRaw = j.kp * eQ + j.intQ + we * (j.Lhat * j.id + j.lambdahat);
  const double vMax = _bus.Vdc / std::sqrt(3.0);
  const double vMag = std::sqrt(vdRaw * vdRaw + vqRaw * vqRaw);
  if (vMag > vMax)
  {
    vdRaw *= vMax / vMag;
    vqRaw *= vMax / vMag;
    j.intD -= j.ki * eD * dt_;  // conditional-integration anti-windup
    j.intQ -= j.ki * eQ * dt_;
  }
  const double vdCmd = j.vdPrev;   // one-sample computation delay
  const double vqCmd = j.vqPrev;
  j.vdPrev = vdRaw;
  j.vqPrev = vqRaw;

  // Inverter: dead-time voltage error per phase, mapped to d-q.
  const double thE = j.p * j.thM;
  const double vDt = _bus.Vdc * (_bus.tDt / dt_);
  const double ia = j.id * std::cos(thE) - j.iq * std::sin(thE);
  const double ib = j.id * std::cos(thE - kTwoPiOver3) -
                    j.iq * std::sin(thE - kTwoPiOver3);
  const double ic = j.id * std::cos(thE + kTwoPiOver3) -
                    j.iq * std::sin(thE + kTwoPiOver3);
  const double ea = -vDt * std::tanh(ia / _bus.epsS);
  const double eb = -vDt * std::tanh(ib / _bus.epsS);
  const double ec = -vDt * std::tanh(ic / _bus.epsS);
  const double vd = vdCmd +
      2.0 / 3.0 * (ea * std::cos(thE) + eb * std::cos(thE - kTwoPiOver3) +
                    ec * std::cos(thE + kTwoPiOver3));
  const double vq = vqCmd -
      2.0 / 3.0 * (ea * std::sin(thE) + eb * std::sin(thE - kTwoPiOver3) +
                    ec * std::sin(thE + kTwoPiOver3));

  // Electrical + rotor ODE, one RK4 step.
  std::array<double, 4> x{j.id, j.iq, j.thM, j.wM};
  x = RK4Electrical(j, x, vd, vq, _thJ, _wJ, dt_);
  j.id = std::clamp(x[0], -200.0, 200.0);
  j.iq = std::clamp(x[1], -200.0, 200.0);
  j.thM = x[2];
  j.wM = x[3];
  j.tauEm = 1.5 * j.p * j.lambda * j.iq;
  return 1.5 * (vd * j.id + vq * j.iq);  // electrical power drawn
}

std::array<double, 4> ActuatorDynamics::RK4Electrical(
    const JointActuator &j, const std::array<double, 4> &x,
    double _vd, double _vq, double _thJ, double _wJ, double _dt)
{
  auto deriv = [&](const std::array<double, 4> &s) {
    const double we = j.p * s[3];
    const double did = (_vd - j.R * s[0] + we * j.L * s[1]) / j.L;
    const double diq =
        (_vq - j.R * s[1] - we * j.L * s[0] - we * j.lambda) / j.L;
    const double phi = s[2] / j.N - _thJ;
    const double phiDot = s[3] / j.N - _wJ;
    const double tauFs =
        j.k0 * phi + j.k2 * phi * phi * phi + j.d * phiDot;
    const double tauEm = 1.5 * j.p * j.lambda * s[1];
    const double dwM = (tauEm - tauFs / j.N -
                        j.pCM * std::tanh(s[3] / j.eps) - j.pvM * s[3]) /
                       j.Jm;
    return std::array<double, 4>{did, diq, s[3], dwM};
  };
  const auto k1 = deriv(x);
  const auto k2 = deriv({x[0] + _dt / 2 * k1[0], x[1] + _dt / 2 * k1[1],
                         x[2] + _dt / 2 * k1[2], x[3] + _dt / 2 * k1[3]});
  const auto k3 = deriv({x[0] + _dt / 2 * k2[0], x[1] + _dt / 2 * k2[1],
                         x[2] + _dt / 2 * k2[2], x[3] + _dt / 2 * k2[3]});
  const auto k4 = deriv({x[0] + _dt * k3[0], x[1] + _dt * k3[1],
                         x[2] + _dt * k3[2], x[3] + _dt * k3[3]});
  return {x[0] + _dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
          x[1] + _dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
          x[2] + _dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]),
          x[3] + _dt / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])};
}

std::array<double, 2> ActuatorDynamics::RK4Mechanical(
    const JointActuator &j, const std::array<double, 2> &x,
    double _tauEm, double _thJ, double _wJ, double _dt)
{
  auto deriv = [&](const std::array<double, 2> &s) {
    const double phi = s[0] / j.N - _thJ;
    const double phiDot = s[1] / j.N - _wJ;
    const double tauFs =
        j.k0 * phi + j.k2 * phi * phi * phi + j.d * phiDot;
    const double dwM = (_tauEm - tauFs / j.N -
                        j.pCM * std::tanh(s[1] / j.eps) - j.pvM * s[1]) /
                       j.Jm;
    return std::array<double, 2>{s[1], dwM};
  };
  const auto k1 = deriv(x);
  const auto k2 = deriv({x[0] + _dt / 2 * k1[0], x[1] + _dt / 2 * k1[1]});
  const auto k3 = deriv({x[0] + _dt / 2 * k2[0], x[1] + _dt / 2 * k2[1]});
  const auto k4 = deriv({x[0] + _dt * k3[0], x[1] + _dt * k3[1]});
  return {x[0] + _dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
          x[1] + _dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])};
}
}  // namespace ur_plant_world

GZ_ADD_PLUGIN(
    ur_plant_world::ActuatorDynamics,
    gz::sim::System,
    ur_plant_world::ActuatorDynamics::ISystemConfigure,
    ur_plant_world::ActuatorDynamics::ISystemPreUpdate)
