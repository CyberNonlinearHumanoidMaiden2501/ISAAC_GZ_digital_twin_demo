// Copyright 2026 Qingyu Geng
//
// Port of the Octave reference model (octave/actuator/):
//   foc_loop.m  -> StepJoint's PI + decoupling + one-sample delay + clamp
//   pmsm.m      -> RK4Electrical (d-q ODE)
//   two_mass.m  -> rotor side of RK4Electrical / RK4Mechanical (the link
//                  side is DART's; only the flexspline coupling touches it)
//   params.m + params_mech.m -> plant_config.yaml `actuator:` table
#include "ur_plant_world/actuator_dynamics.hh"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/common/Console.hh>
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
  if (act["Vdc"])
    Vdc_ = act["Vdc"].as<double>();
  if (act["wc"])
    wc_ = act["wc"].as<double>();
  const double errR = act["id_err"]["R"].as<double>();
  const double errL = act["id_err"]["L"].as<double>();
  const double errKt = act["id_err"]["Kt"].as<double>();

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

void ActuatorDynamics::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  const double simTime =
      std::chrono::duration<double>(_info.simTime).count();
  const bool logNow = simTime - lastLogSimTime_ >= logPeriod_;

  for (auto &j : joints_)
  {
    // Command (set by gz_ros2_control's PreUpdate, which runs before ours).
    double tauJCmd = 0.0;
    const auto *forceCmd =
        _ecm.Component<gz::sim::components::JointForceCmd>(j.jointEntity);
    if (forceCmd && !forceCmd->Data().empty())
      tauJCmd = forceCmd->Data()[0];

    // DART joint state (the link side).
    double thJ = 0.0, wJ = 0.0;
    const auto *pos =
        _ecm.Component<gz::sim::components::JointPosition>(j.jointEntity);
    const auto *vel =
        _ecm.Component<gz::sim::components::JointVelocity>(j.jointEntity);
    if (pos && !pos->Data().empty())
      thJ = pos->Data()[0];
    if (vel && !vel->Data().empty())
      wJ = vel->Data()[0];

    if (actuatorModel_ == "l0")
    {
      if (logNow)
        gzdbg << "actuator l0 passthrough: '" << j.name
              << "' cmd=" << tauJCmd << " Nm\n";
      continue;  // passthrough: the command flows to the physics unchanged
    }

    // Rotor initialization: zero initial windup against the current pose.
    if (!j.rotorInit)
    {
      j.thM = j.N * thJ;
      j.wM = j.N * wJ;
      j.rotorInit = true;
    }

    for (int s = 0; s < substeps_; ++s)
      StepJoint(j, tauJCmd, thJ, wJ);

    // Resolved joint torque: flexspline minus the Gnad joint-side friction.
    const double phi = j.thM / j.N - thJ;
    const double phiDot = j.wM / j.N - wJ;
    j.tauFs = j.k0 * phi + j.k2 * phi * phi * phi + j.d * phiDot;
    const double tauJ = j.tauFs - (j.pC * std::tanh(wJ / j.eps) + j.pv * wJ);
    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(
        j.jointEntity, std::vector<double>{tauJ});

    if (logNow)
    {
      gzdbg << "actuator: '" << j.name << "' cmd=" << tauJCmd
            << " Nm -> tau_fs=" << j.tauFs << " tau_j=" << tauJ
            << " i_q=" << j.iq << " windup=" << phi << " rad\n";
    }
  }
  if (logNow)
    lastLogSimTime_ = simTime;
}

void ActuatorDynamics::StepJoint(JointActuator &j, double _tauJCmd,
    double _thJ, double _wJ)
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
    return;
  }

  // l2: FOC current loop (belief gains/decoupling, one-sample delay,
  // circular clamp at the nominal bus).
  const double we = j.p * j.wM;
  const double iqStar = tauMStar / j.Kthat;
  const double eD = -j.id;
  const double eQ = iqStar - j.iq;
  j.intD += j.ki * eD * dt_;
  j.intQ += j.ki * eQ * dt_;
  double vdRaw = j.kp * eD + j.intD - we * j.Lhat * j.iq;
  double vqRaw = j.kp * eQ + j.intQ + we * (j.Lhat * j.id + j.lambdahat);
  const double vMax = Vdc_ / std::sqrt(3.0);
  const double vMag = std::sqrt(vdRaw * vdRaw + vqRaw * vqRaw);
  if (vMag > vMax)
  {
    vdRaw *= vMax / vMag;
    vqRaw *= vMax / vMag;
    j.intD -= j.ki * eD * dt_;  // conditional-integration anti-windup
    j.intQ -= j.ki * eQ * dt_;
  }
  const double vd = j.vdPrev;   // one-sample computation delay
  const double vq = j.vqPrev;
  j.vdPrev = vdRaw;
  j.vqPrev = vqRaw;

  // Electrical + rotor ODE, one RK4 step.
  std::array<double, 4> x{j.id, j.iq, j.thM, j.wM};
  x = RK4Electrical(j, x, vd, vq, _thJ, _wJ, dt_);
  j.id = x[0];
  j.iq = x[1];
  j.thM = x[2];
  j.wM = x[3];
  j.tauEm = 1.5 * j.p * j.lambda * j.iq;
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
