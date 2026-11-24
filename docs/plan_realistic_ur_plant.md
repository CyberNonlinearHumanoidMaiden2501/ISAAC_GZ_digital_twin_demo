# Plan: Realistic UR Arm "Plant" in Gazebo (overlay packages `ur_plant_*`)

> Status: DRAFT v2 — revised 2026-10-25 from hardware research
> ([`ur_hardware_research.md`](ur_hardware_research.md), sources in [`sources/`](sources/README.md)).
> Not yet executed.
> Scope: make the simulated UR robot arm behave like a real robot (motor dynamics,
> friction, sensor noise, real-world faults) so it can serve as the "physical plant"
> for a digital twin running in Isaac Sim.

## Goal

Turn the stock UR Gazebo sim into a realistic plant for digital-twin learning,
without touching the pristine `ur_simulation_gz` clone. The plant gets:

- torque-driven joints with servo-like actuator dynamics,
- joint friction and acceleration limits,
- noisy encoders, a TCP force-torque sensor, a base 6-axis IMU and a tool-flange
  accelerometer (all at real 500 Hz rates),
- a runtime fault-injection API grounded in documented UR error codes.

It publishes noisy sensor topics for the Isaac twin to consume and keeps clean
ground truth on a separate topic for validation.

## Current state (findings from exploration)

- ROS 2 Jazzy + Gazebo Harmonic 8.15, `gz_ros2_control` 1.2.20, `joint_trajectory_controller` 4.42.1.
- The stock sim is ideal in exactly the ways a real arm is not:
  - joints are **position-commanded** (no torque loop, no PID gains anywhere),
  - `<dynamics damping="0" friction="0"/>` hardcoded on every joint
    (`ur_macro.xacro:298-366` in binary `ur_description` 3.5.1),
  - **no acceleration limits** (`has_acceleration_limits: false` in every `joint_limits.yaml`),
  - **no sensors** (no IMU, no F/T; `ur_sensors.xacro` is only wired into the mock-hardware path),
  - no noise, no fault mechanisms.
- Effort command interfaces **are already declared** on all 6 joints
  (`ur_joint_control.xacro`), so torque control needs no URDF hardware changes.
- JTC 4.42.1 supports `command_interfaces: [effort]` with per-joint PID gains — installed.
- World shadowing hazard: workspace overlay prepends the wheelchair demo's
  `GZ_SIM_RESOURCE_PATH`, so default `empty.sdf` resolves to a world with
  5 ms steps / RTF 10 instead of the stock 1 ms / RTF 1 world.
- Initial joint positions are hardcoded in `ur_gz.ros2_control.xacro` (0, -1.57, 0, -1.57, 0, 0).
- `ur_simulation_gz` clone is pristine (branch `jazzy`, commit `3600c68`).
- No gripper description installed anywhere (out of scope for now).

## Decisions (confirmed with user)

| Decision | Choice |
|---|---|
| Placement | **New overlay packages** `ur_plant_*`; upstream clone stays pristine |
| Motor model | **Effort JTC with PID + L2 actuator plugin** (PMSM electrical cascade + two-mass flexspline; L1/L0 kept as config fallbacks — decided 2026-11-02) |
| Sensors | **Noisy encoders + TCP F/T + base 6-axis IMU + tool-flange accelerometer** |
| Faults | **Runtime fault-injection API** (ROS 2 services), grounded in documented UR error codes |
| Stop behavior | **Deferred** — faults expose a `severity` field; protective-stop / safety state machine is a later phase |
| Package layout | **Four packages built from the existing gazebosim project template**: `ur_plant_description`, `ur_plant_world`, `ur_plant_bringup`, `ur_plant_sensors` |

## Package structure — four packages, built from the existing gazebosim project template

The three existing `ros_gz_*` packages in `src/` are renamed copies of the official
gazebosim `ros_gz_project_template` (package names `ros_gz_wheelchair_*`, the template's
example C++ namespace still inside, plugin build targets commented out, content
directories empty). They are repurposed and renamed, plus one new package:

```
src/
├── ur_plant_description/             # (from ros_gz_description) plant config + description
│   ├── config/plant_config.yaml      # friction, accel limits, encoder/motor/IMU params,
│   │                                 #   per-model tables (ur3e/ur5e/ur10e/ur16e)
│   ├── models/<ur_type>/ur_plant.urdf + model.config   # generated once, hand-edited, committed; model://-resolvable
│   └── models/payload.sdf            # configurable-mass payload box for payload_attach
├── ur_plant_world/                   # (from ros_gz_world) worlds + gz-sim system plugins
│   ├── worlds/ur_plant_world.sdf     # pinned DART physics + our two plugins
│   ├── include/ur_plant_world/       # plugin headers (namespace ur_plant_world)
│   └── src/
│       ├── actuator_dynamics.cpp     # motor lag, torque rate limit, saturation
│       └── fault_injection.cpp       # mechanical/electrical faults + payload attach (links rclcpp)
├── ur_plant_bringup/                 # (from ros_gz_bringup) launch + ROS config
│   ├── launch/ur_plant.launch.py     # superset of upstream ur_sim_control.launch.py
│   └── config/ur_plant_controllers.yaml   # effort-mode JTC with per-joint PID gains
└── ur_plant_sensors/                 # (new) ROS-side plant logic
    ├── srv/SetFault.srv              # {fault, joint, value} → {success, message}
    ├── src/plant_sensors.cpp         # encoder/IMU/tool-accel handling + sensor fault service
    └── scripts/inject_fault.py       # CLI helper for the fault services
```

Dependency edges: `ur_plant_bringup` → the other three; `ur_plant_world` →
`ur_plant_sensors` (for `SetFault.srv` only, while linking rclcpp to serve
`/plant/mechanical_faults` directly, the same pattern `gz_ros2_control` uses);
`ur_plant_description` → `ur_description` and `xacro` only.

**Template fixes applied during the rename:** package.xml `<name>`s, CMake
`project()`/`find_package`s/install destinations, hook filenames, the C++ namespace
`ros_gz_example_gazebo` → `ur_plant_world`, the include dir and the READMEs are renamed;
the deprecated `IGNITION_ADD_PLUGIN` macro is replaced by `GZ_ADD_PLUGIN`; env hooks are
made consistent (the description `.dsv`/`.sh` disagreed on what to prepend, and the
world `.sh` set the legacy `GZ_SIM_PLUGIN_PATH` while its `.dsv` set
`GZ_SIM_SYSTEM_PLUGIN_PATH` — both now prepend the models/worlds resource dir and
`GZ_SIM_SYSTEM_PLUGIN_PATH`); `worlds/empty.sdf` is deleted (side benefit: stock
`ur_simulation_gz` launches then resolve gz-sim's real 1 ms `empty.sdf` instead of the
shadowing 5 ms / RTF 10 world); stale `build/ros_gz_wheelchair_*` and
`install/ros_gz_wheelchair_*` prefixes are removed before a fresh colcon build, since the
workspace `install/setup.bash` currently chains their env hooks.

## Components

### 1. `ur_plant_world` — `worlds/ur_plant_world.sdf`

- DART physics: `max_step_size=0.001`, `real_time_factor=1.0`, `real_time_update_rate=1000`.
- Ground plane + sun; physics / user-commands / scene-broadcaster / contact systems.
- Loads our two plugins: `actuator_dynamics` and `fault_injection`.
- Passed as an **absolute path** in launch to defeat the `GZ_SIM_RESOURCE_PATH`
  shadowing bug described above.

### 2. `ur_plant_bringup` — `config/ur_plant_controllers.yaml`

- Copy of upstream `ur_controllers.yaml` with the trajectory controller switched to
  `command_interfaces: [effort]` and per-joint `gains: {p, i, d, i_clamp}`.
- State interfaces become position + velocity (JTC's `state_interfaces` only accepts
  `{position, velocity, acceleration}`; effort still flows to `/raw_joint_states` via
  the broadcaster's state read); controller_manager stays at 500 Hz.
- **JSB publishes at the full 500 Hz** (no `state_publish_rate` override) — matching the
  real UR driver behavior.
- Starting gains based on joint inertia (e.g., shoulder ~p=4000/i=400/d=50,
  wrist ~p=800/i=50/d=10); tuned once in Stage 2 against the complete plant
  (step response: rise time, overshoot vs real UR servo bandwidth).
- Keep upstream tolerances/constraints; may tighten later.

### 3. `ur_plant_description` — the committed plant URDF

- The base URDF is generated **once** from the upstream `ur_gz.urdf.xacro` (ur_description
  macros, world/ground links, `gz_ros2_control` plugin block), committed to
  `models/<ur_type>/ur_plant.urdf`, then hand-edited:
  - per-joint `<dynamics damping/friction>` → viscous + Coulomb values from
    `plant_config.yaml`,
  - `initial_value` params from `initial_positions`,
  - three `<gazebo>` sensor blocks:
    - **force_torque sensor on `wrist_3_joint`** — per-axis `<noise type="gaussian">`
      (stddev, bias_mean, bias_stddev, dynamic bias), 500 Hz, saturated at the vendor
      ±50 N / ±10 Nm;
    - **6-axis IMU at `base_link`** — per-axis gaussian noise on angular_velocity and
      linear_acceleration, 500 Hz (represents the real Base Filter Board IMUs);
    - **IMU at `tool0` used as accelerometer only** — mirrors the real Tool PCBA
      accelerometer (RTDE `actual_tool_accelerometer`); gyro part discarded.
- No generator script, no launch-time code generation; regeneration is a documented
  manual step (see `plan_stage1_description.md`). `robot_state_publisher` reads the
  file; `ros_gz_sim create` spawns with `-file`.

### 4. `ur_plant_sensors` — `plant_sensors.cpp` (rclcpp node)

- JSB remapped to `/raw_joint_states` (clean = ground truth, republished explicitly
  as `/ground_truth/joint_states`).
- Publishes `/plant/joint_states` (500 Hz) with:
  - encoder quantization **~1e-5 rad** (derived from 1080 CPR motor encoder through
    101:1 and the AksIM output encoder; UR publishes no official bit depth),
  - gaussian noise **σ ≈ 0.5–1e-5 rad** **[estimate — no published measurement]**,
  - slow bias random-walk drift,
  - optional latency.
- Handles the F/T stream: publishes `/plant/ft_sensor/wrench` (500 Hz) with the
  vendor precision/accuracy envelopes (±3.5 N / ±0.2 Nm precision, ±4.0 N / ±0.3 Nm
  accuracy), noise default **σ_F = 1.0 N, σ_T = 0.05 Nm** (vendor-anchored, conservative;
  literature option 0.3 N + 100 Hz LP selectable in config), and **relative-sensor
  semantics**: zeroing service `/plant/ft_sensor/zero` (mirrors the driver's
  `~/zero_ftsensor`) + slow bias drift.
- Publishes `/plant/tool_accel` (500 Hz) as `sensor_msgs/Imu` with only
  `linear_acceleration` populated (orientation/angular_velocity covariances = -1).
- Hosts service `/plant/sensor_faults`:
  - `encoder_fault` (C75A/C265A) — dropout, bias jump, spike, saturation,
  - `fts_zero_shift` (C450) — step bias in wrench until re-zeroed,
  - `imu_fault` (C353) — bias/drift/dropout on IMU and tool accel,
  - `comms_latency` (C49A/C76A) — added latency/jitter on sensor topics,
  - generic noise multiplier and dropout (stale/cold topics).

### 5. `ur_plant_world` — `actuator_dynamics.cpp` (gz-sim system plugin)

Full per-joint actuator cascade (**L2**, decided 2026-11-02): a PMSM electrical
stage driving a two-mass flexspline model. The plugin keeps a per-joint **motor
state** (rotor angle/velocity, phase current, bus voltage), integrates it at the
world's 1 ms step, and applies the resulting joint-side torque to DART — DART
never sees the stiff gear coupling, only the resolved torque.

Actuator modeling levels (the ladder this component implements; `l1`/`l0` are
config fallbacks in `plant_config.yaml`):

| Level | Model | Captures | Misses |
|---|---|---|---|
| **L0** | first-order lag + torque rate limit + saturation | gross servo bandwidth, saturation | resonance, ripple, windup, motor-side states |
| **L1** | L0 + rotor ODE + stiffening flexspline k(φ) | two-mass resonance, windup, motor-vs-joint encoder divergence, motor-side friction | electrical detail (current loop, back-EMF, PWM) |
| **L2** (chosen) | L1 + PMSM electrical stage (current loop, back-EMF, PWM delay, cogging, bus model) | electrical-fault signatures (voltage sag, overcurrent, regen) | semiconductor-level inverter detail |

- **PMSM electrical stage** (per joint):
  - voltage balance `v = R·i + L·di/dt + k_e·ω_m`, torque `τ_em = k_t·i`;
    R/L/k_t/k_e from the candidate Kollmorgen TBM2G motor datasheets [OEM]
    (UR has never publicly confirmed the motor series; τ_e = L/R ≈ 0.9 ms
    wrist / 1.3 ms J1–3 [L]);
  - **PI current loop** (~1 kHz bandwidth, tuned) + **PWM transport delay**
    (one switching period, 10–20 kHz — not published, estimate);
  - **cogging/torque ripple** as Fourier terms at pole-pair multiples
    (kinematic-error form from the Madsen thesis [L] + OEM pole/slot data);
  - **48 V-class DC bus** with regen clamp at 55 V (C56A): `voltage_sag`
    (C296A44/C50A51) and overcurrent (C59A) faults act on V_bus and the
    current limit, making the electrical fault family physically implementable.
- **Two-mass flexspline stage**: rotor ODE
  `J_m·ω̇_m = τ_em − τ_fs/N − τ_fric,motor` (J_m = 6.4e-5 / 1.2e-5 kg·m² [L])
  coupled to the joint through the stiffening spring `τ_fs = k(φ)·φ + d·φ̇`,
  windup `φ = θ_m/N − θ_j`, gear ratio N = 101 [L]; k(φ) from the Madsen
  thesis UR5e identification (values to be extracted from the thesis figures).
- **Friction in physical Nm lives here** — joint-side Coulomb/viscous (from
  `plant_config.yaml`) plus motor-side velocity/load/temperature-dependent
  friction [L]; the unitless URDF `<dynamics friction>` is neutralized while
  the plugin is active (Stage 1's URDF values are interim).
- **Saturation at the vendor joint torque ratings** (150/150/150/28/28/28 Nm)
  enforced as the current/torque limit; no public continuous/peak distinction.
- Reads per-joint fault scale factors (effort derate, brake drag, jam braking
  torque) from the fault plugin.
- Config fallbacks: `actuator_model: l1` = two-mass without the electrical
  stage; `l0` = the original first-order lag + rate limit + saturation (kept
  for A/B experiments).
- Fallback if `JointForceCmd` interception conflicts with `gz_ros2_control`
  write timing: apply corrective forces in PreUpdate.

### 6. `ur_plant_world` — `fault_injection.cpp` (gz-sim system plugin, links rclcpp)

- Service `/plant/mechanical_faults` (`SetFault.srv`), faults grounded in documented
  error codes, each with a `severity` field (warning/protective/stop) so the deferred
  stop state machine can be layered on later:

| Fault | Real mechanism / code | Injectable effect |
|---|---|---|
| `gearbox_wear` | harmonic drive degradation (C46A, C70A, recurrent C153) | friction ramp + small backlash + torque ripple |
| `joint_jam` | obstruction/stall (C157, C59A) | velocity forced to 0 / braking torque |
| `brake_drag` | partially engaged brake (C57A, C62A1) | extra constant drag + heat input |
| `torque_derate` | thermal derating / weak motor | effort scale factor < 1 |
| `payload_misconfig` | wrong payload/CoG (C155/C156/C162) | attach offset-mass payload |
| `collision_push` | external contact (C157/C158, C191A6/A7/A12) | impulsive/sustained external force on TCP |
| `voltage_sag` | 24 V rail brownout (C296A44, C50A51) | reduced torque capacity + comms glitches |
| `joint_overheat` | duty-cycle thermal model (C62A1/A11) | first-order temp rise; warning 80 °C, shutdown 85 °C, no auto-recovery |

- Sensor-side faults (`encoder_fault`, `fts_zero_shift`, `imu_fault`, `comms_latency`)
  live in `plant_sensors.cpp` (component 4).
- `payload_attach/detach`: spawn a configurable-mass box attached to `tool0` via
  DetachableJoint.

### 7. `ur_plant_bringup` — `launch/ur_plant.launch.py`

- Preserves upstream arguments (`ur_type` default ur5e, `safety_limits`, `gazebo_gui`,
  `activate_joint_controller`).
- Adds: absolute world path, generated description, our controllers file, spawn of
  sensor node, and `ros_gz_bridge` bridges for the F/T sensor, base IMU and tool
  accelerometer, JSB remap.
- Headless mode supported for CI.

## Topic contract (what the Isaac twin will see)

| Topic | Meaning | Rate |
|---|---|---|
| `/ground_truth/joint_states` | clean sim truth — for twin validation only, never for estimation | 500 Hz |
| `/plant/joint_states` | noisy, quantized encoder data | 500 Hz |
| `/plant/ft_sensor/wrench` | TCP force-torque, ±50 N / ±10 Nm saturation, relative (zeroing) | 500 Hz |
| `/plant/imu` | base 6-axis IMU with noise/bias | 500 Hz |
| `/plant/tool_accel` | tool-flange accelerometer (linear only) | 500 Hz |
| `/joint_trajectory_controller/follow_joint_trajectory` | commanded trajectories | — |

## Parameter grounding (UR5e defaults; per-model tables in `plant_config.yaml`)

| Plant parameter | Value | Provenance |
|---|---|---|
| Effort saturation | 150/150/150/28/28/28 Nm | [V] UR "Max. joint torques" |
| Velocity limits | 3.1416 rad/s (180 °/s) all joints | [V] user manual |
| Gear ratio | 101:1 | [L] |
| Rotor inertia J_m | 6.4e-5 (J1–3) / 1.2e-5 (J4–6) kg·m² | [L] Robotica 2024 |
| Motor electrical τ_e = L/R | 0.9 ms wrist / 1.3 ms J1–3 | [L] derived from candidate-motor datasheets |
| Flexspline stiffness k(φ) | stiffening spring; values to extract from thesis figures | [L] Madsen |
| Joint friction | Gnad 2026 per-joint table: Coulomb 8.58/12.94/9.56/2.24/2.08/2.13 Nm, viscous 13.9/8.9/8.92/1.26/1.59/1.39 N·m·s/rad | [L] |
| Encoder quantization | ~1e-5 rad | derived estimate |
| Encoder noise | σ ≈ 0.5–1e-5 rad | estimate [NF] |
| FTS range / precision / accuracy | ±50 N / ±10 Nm; ±3.5 N / ±0.2 Nm; ±4.0 N / ±0.3 Nm | [V] datasheet |
| IMU full-scale / noise | ±16 g / ±2000 dps; σ ≈ 5–10 mg, 0.02–0.05 dps | [OEM] chip specs (UR publishes none) |
| Thermal thresholds | log >50 °C, warn 80 °C, shutdown 85 °C | [V] error codes |

## Implementation stages

- **Stage 0 (repackage):** rename the three template packages to `ur_plant_*`, create
  `ur_plant_sensors`, apply the template fixes (namespace, hooks, `GZ_ADD_PLUGIN`,
  delete `worlds/empty.sdf`), remove stale `build/`/`install/` prefixes, and do a fresh
  colcon build of the four named-but-empty packages.
- **Stage 1 (config-level realism, no new C++):** description (committed hand-edited
  URDF generated once from the upstream xacro, plant_config, payload model) → world
  (pinned SDF) → bringup (effort+PID controllers, launch file). Detailed design:
  [`plan_stage1_description.md`](plan_stage1_description.md).
  Verify headless: `example_move.py` works, tracking error behaves like a
  real arm (finite settling, gravity-holding efforts on the lift joints), tune PID.
- **Stage 2 (C++ realism):** actuator modeled and tested **in Octave first**
  (reference model in `octave/actuator/`), then ported to the world plugins
  (`actuator_dynamics` — the L2 electrical +
  two-mass cascade above; `fault_injection`) → sensors package (`plant_sensors` node,
  `SetFault.srv`, sensor faults, FTS zeroing, `inject_fault.py` CLI) → thermal model →
  **tune the servo emulation (PID) once, against the complete plant (actuator +
  friction)** — acceptance: `example_move.py` completes without tolerance aborts,
  noting the Stage 1 abort was partly torque-saturation-limited so friction
  calibration may be part of the fix → bridges → README documenting every fault and
  its expected signature. Research sub-tasks feeding the plugin: extract k(φ) from
  the Madsen thesis figures (in `docs/sources/literature/`), source R/L/k_t/k_e from
  the Kollmorgen TBM2G datasheets, and the Gnad per-joint friction table
  (obtained 2026-11-24). Detailed design: [`plan_stage2_realism.md`](plan_stage2_realism.md).
- **Stage 3 (validation):** smoke test modeled on upstream `test_gz.py` (launch
  headless, move, assert topics) + fault-signature checks (jam → velocity zero +
  effort spike; dropout → stale stamps; derate → larger tracking error).
- **Deferred (future phase):** protective-stop vs Cat-0 safety state machine, stop
  categories 0/1/2, Recovery mode limits, C163 unlock throttle — enabled by the
  `severity` field without breaking the fault API.

## Out of scope (can be added later)

- Protective-stop / safety state machine (deferred phase, see above).
- Motor thermal-electrical coupling (winding R(T) feedback into the electrical
  stage) — the thermal model stays first-order and one-way.
- Semiconductor-level inverter detail (switching transients, dead-time) — PWM is
  modeled as a transport delay only.
- Thermal physics beyond a first-order duty-cycle model.
- Gripper (no gripper description installed).
- Wrist camera, base vibration model, cable drag.
- Contact-rich environments with objects.
- Action-level latency emulation.

## Risks

- PID/effort + current-loop tuning is empirical and happens in Stage 2 once the L2
  actuator exists; budget one tuning session; values will be refined against real
  data later.
- Per-joint friction is now the Gnad 2026 table (UR5e mean of two robots — see
  open questions); the residual uncertainty is robot-to-robot spread and the
  asymmetric-Coulomb direction split — accepted as approximate for the twin.
- The L2 electrical parameters (R, L, k_t, k_e, PWM frequency) come from
  *candidate* Kollmorgen motors UR has never publicly confirmed — the electrical
  stage models the right class of servo, not the exact unit; keep them
  config-tunable and provenance-tagged [OEM]/estimate.
- IMU parameters are chip-level specs (ISM330IS / ICM42688), not UR-published
  robot values — marked as estimates.
- Stage 2 moves all friction into the actuator plugin in physical Nm (motor-side +
  joint-side), so DART's unitless URDF-friction interpretation stops mattering once
  the plugin is active; the Stage 1 URDF values are interim.
- Upstream updates stay trivial: nothing in `ur_simulation_gz` is modified.

## Open questions for refinement

- [x] IMU scope — decided: base 6-axis IMU + tool-flange accelerometer.
- [x] Stop state machine — deferred to a later phase.
- [x] Exact per-joint friction/damping split — RESOLVED 2026-11-24: the Gnad
      2026 per-joint table was obtained (`docs/sources/literature/friction_table.xlsx`,
      UR5e mean of two robots). Viscous: 13.9/8.9/8.92/1.26/1.59/1.39 N·m·s/rad;
      Coulomb: 8.58/12.94/9.56/2.24/2.08/2.13 Nm. The asymmetric-Coulomb
      direction split (forward vs backward power flow) remains a later
      refinement.
- [x] FTS noise anchor — decided 2026-11-02: **vendor-conservative (worst-case)**
      values stay the default (σ_F = 1.0 N, σ_T = 0.05 Nm); the literature model
      remains a config option. Noise being "a bit off" is acceptable.
- [x] Actuator modeling depth — decided 2026-11-02: **L2 full electrical** (PMSM
      current loop + back-EMF + PWM transport delay + cogging) driving two-mass
      flexspline dynamics, in `actuator_dynamics.cpp`; L1 (two-mass only) and L0
      (first-order lag) kept as config fallbacks. Rationale: gearbox-wear ripple,
      motor-vs-joint encoder divergence, voltage-sag and overcurrent faults are
      motor-side phenomena a lag filter cannot produce, and the twin pipeline
      needs structurally real actuator dynamics — this is the one place the model
      must not be approximate.
- [ ] Fault list priority: which faults matter most for the twin experiments?
- [ ] Payload: fixed catalog of masses (e.g., 0.5 / 1 / 2 kg) or arbitrary value?
- [ ] Should the plant also simulate the UR driver's TCP/UART comms latency
      (i.e., delay between controller command and joint actuation)?
