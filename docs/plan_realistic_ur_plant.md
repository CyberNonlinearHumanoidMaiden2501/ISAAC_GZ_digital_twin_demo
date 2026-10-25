# Plan: Realistic UR Arm "Plant" in Gazebo (overlay package `ur_plant_gz`)

> Status: DRAFT — not yet executed. Saved for refinement.
> Scope: make the simulated UR robot arm behave like a real robot (motor dynamics,
> friction, sensor noise, real-world faults) so it can serve as the "physical plant"
> for a digital twin running in Isaac Sim.

## Goal

Turn the stock UR Gazebo sim into a realistic plant for digital-twin learning,
without touching the pristine `ur_simulation_gz` clone. The plant gets:

- torque-driven joints with servo-like actuator dynamics,
- joint friction and acceleration limits,
- noisy encoders + a TCP force-torque sensor,
- a runtime fault-injection API.

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
| Placement | **New overlay package** `ur_plant_gz`; upstream clone stays pristine |
| Motor model | **Effort JTC with PID + custom actuator plugin** (two stages) |
| Sensors | **Noisy encoders + TCP F/T sensor** (no IMU — not standard on real UR) |
| Faults | **Runtime fault-injection API** (ROS 2 services) |

## New package: `src/ur_plant_gz` (ament_cmake, colcon)

```
src/ur_plant_gz/
├── package.xml / CMakeLists.txt      # deps: rclcpp, gz-sim, ros_gz_bridge, ros_gz_sim,
│                                     #       joint_trajectory_controller, ur_description, xacro...
├── config/
│   ├── ur_plant_controllers.yaml     # effort-mode JTC with per-joint PID gains
│   └── plant_config.yaml             # friction, accel limits, encoder noise, motor params
├── worlds/ur_plant_world.sdf         # pinned DART physics + our two plugins
├── urdf/ur_plant.urdf.xacro          # upstream structure + TCP F/T sensor block
├── scripts/generate_plant_description.py  # xacro → patched URDF (dynamics, limits)
├── srv/SetFault.srv                   # {fault, joint, value} → {success, message}
├── include/ur_plant_gz/ + src/        # plugins + node (C++)
│   ├── actuator_dynamics.cpp          # motor lag, torque rate limit, saturation
│   ├── fault_injection.cpp            # mechanical faults + payload attach
│   └── plant_sensors.cpp              # encoder noise + sensor fault service
├── launch/ur_plant.launch.py          # superset of upstream ur_sim_control.launch.py
└── scripts/inject_fault.py            # CLI helper for the fault services
```

## Components

### 1. `worlds/ur_plant_world.sdf`

- DART physics: `max_step_size=0.001`, `real_time_factor=1.0`, `real_time_update_rate=1000`.
- Ground plane + sun; physics / user-commands / scene-broadcaster / contact systems.
- Loads our two plugins: `actuator_dynamics` and `fault_injection`.
- Passed as an **absolute path** in launch to defeat the `GZ_SIM_RESOURCE_PATH`
  shadowing bug described above.

### 2. `config/ur_plant_controllers.yaml`

- Copy of upstream `ur_controllers.yaml` with the trajectory controller switched to
  `command_interfaces: [effort]` and per-joint `gains: {p, i, d, i_clamp}`.
- State interfaces become position + velocity + effort; controller_manager stays at 500 Hz.
- Starting gains based on joint inertia (e.g., shoulder ~p=4000/i=400/d=50,
  wrist ~p=800/i=50/d=10), tuned empirically in a dedicated tuning session
  (step response: rise time, overshoot vs real UR servo bandwidth).
- Keep upstream tolerances/constraints; may tighten later.

### 3. URDF generation

- `ur_plant.urdf.xacro` mirrors upstream `ur_gz.urdf.xacro` (ur_description macros,
  world/ground links, `gz_ros2_control` plugin block) and adds a
  `<gazebo reference="tool0">` force_torque sensor block with per-axis
  `<noise type="gaussian">` (stddev, bias_mean, bias_stddev, dynamic bias) at 100 Hz.
- `generate_plant_description.py` runs xacro, then patches the XML from `plant_config.yaml`:
  - per-joint `<dynamics damping/friction>` → viscous + Coulomb values,
  - `has_acceleration_limits: true` with realistic accel limits.
- Launch runs the generator once, then feeds the same patched URDF to both
  `robot_state_publisher` and `ros_gz_sim create`.

### 4. `plant_sensors.cpp` (rclcpp node)

- JSB remapped to `/raw_joint_states` (clean = ground truth, republished explicitly
  as `/ground_truth/joint_states`).
- Publishes `/plant/joint_states` with:
  - encoder quantization (~2π/2^17 rad per joint),
  - gaussian noise,
  - slow bias random-walk drift,
  - optional latency.
- Hosts service `/plant/sensor_faults`:
  - dropout (stale/cold topics),
  - bias offset,
  - drift rate,
  - noise multiplier,
  - added latency.

### 5. `actuator_dynamics.cpp` (gz-sim system plugin)

- PreUpdate filter on each joint's `JointForceCmd` set by `GazeboSimSystem`:
  - first-order motor lag (τ_motor per joint, ~5–20 ms),
  - torque rate limit (dτ/dt cap),
  - saturation to per-joint continuous/peak effort limits.
- Reads a per-joint effort scale factor set by the fault plugin.
- Fallback if `JointForceCmd` interception conflicts with `gz_ros2_control` write
  timing: apply corrective forces in PreUpdate.

### 6. `fault_injection.cpp` (gz-sim system plugin, links rclcpp)

- Service `/plant/mechanical_faults` (`SetFault.srv`):
  - `joint_jam` — zero velocity / braking torque,
  - `friction_boost` — extra Coulomb + viscous torque,
  - `torque_derate` — scale factor on commanded effort,
  - `motor_lag` — τ_motor increase (bearing wear),
  - `payload_attach/detach` — spawn a configurable-mass box attached to tool0
    via DetachableJoint.

### 7. `launch/ur_plant.launch.py`

- Preserves upstream arguments (`ur_type` default ur5e, `safety_limits`, `gazebo_gui`,
  `activate_joint_controller`).
- Adds: absolute world path, generated description, our controllers file, spawn of
  sensor node + F/T sensor bridge (`ros_gz_bridge` mapping gz `Wrench` →
  `/plant/ft_sensor/wrench`; header added by our node if the stamped mapping is
  unavailable), JSB remap.
- Headless mode supported for CI.

## Topic contract (what the Isaac twin will see)

| Topic | Meaning |
|---|---|
| `/ground_truth/joint_states` | clean sim truth — for twin validation only, never for estimation |
| `/plant/joint_states` | noisy, quantized encoder data |
| `/plant/ft_sensor/wrench` | TCP force-torque with noise/bias |
| `/joint_trajectory_controller/follow_joint_trajectory` | commanded trajectories |

## Implementation stages

- **Stage 1 (config-level realism, no new C++):** package skeleton → pinned world →
  effort+PID controllers → URDF generator with friction/accel limits + F/T sensor →
  launch file. Verify headless: `example_move.py` works, tracking error behaves like a
  real arm (finite settling, gravity-holding efforts on the lift joints), tune PID.
- **Stage 2 (C++ realism):** `plant_sensors` node → `actuator_dynamics` plugin →
  `fault_injection` plugin + `SetFault.srv` → payload attach → F/T bridge →
  `inject_fault.py` CLI + README documenting every fault and its expected signature.
- **Stage 3 (validation):** smoke test modeled on upstream `test_gz.py` (launch
  headless, move, assert topics) + fault-signature checks (jam → velocity zero +
  effort spike; dropout → stale stamps; derate → larger tracking error).

## Out of scope (can be added later)

- Gripper (no gripper description installed).
- Wrist camera.
- Base vibration model.
- Cable drag.
- Contact-rich environments with objects.
- Action-level latency emulation.

## Risks

- PID/effort tuning is empirical; budget one tuning session; values will be refined
  against real data later.
- DART's interpretation of URDF friction vs physical Nm Coulomb needs a calibration
  check in Stage 1; plugin-based friction (Stage 2) is the controllable fallback.
- Upstream updates stay trivial: nothing in `ur_simulation_gz` is modified.

## Open questions for refinement

- [ ] Exact per-joint friction/damping values (viscous + Coulomb) to start from.
- [ ] Encoder resolution / noise stddev / drift rates — pick from real UR specs.
- [ ] TCP F/T sensor: which vendor model to emulate (e.g., Robotiq FT-300)?
- [ ] Fault list priority: which faults matter most for the twin experiments?
- [ ] Payload: fixed catalog of masses (e.g., 0.5 / 1 / 2 kg) or arbitrary value?
- [ ] Should the plant also simulate the UR driver's TCP/UART comms latency
      (i.e., delay between controller command and joint actuation)?
