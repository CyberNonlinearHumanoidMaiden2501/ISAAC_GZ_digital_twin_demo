# Stage 1 Plan: Description Files (`ur_plant_description`)

> Status: design ready for implementation — saved 2026-10-25, not yet executed.
> Parent plan: [`plan_realistic_ur_plant.md`](plan_realistic_ur_plant.md).
> Scope: the robot description (URDF/xacro + generator + config) and its launch
> integration for Stage 1 (config-level realism, no new C++).

## Design principle

Generate the base URDF **once** from the upstream `ur_simulation_gz` xacro and commit it as a regular, hand-edited file — no generator script, no launch-time code generation. The committed URDF is edited by hand like any URDF written from scratch, with `plant_config.yaml` as the record of the values it contains.

## Decisions (confirmed with user)

| Decision | Choice |
|---|---|
| F/T sensor placement | **`wrist_3_joint`** — gz-sim F/T sensors must live on a joint, and fixed joints (like `wrist_3_link-ft_frame`) may be lumped away in URDF→SDF conversion |
| Sensor blocks | **Authored directly in the committed URDF** as `<gazebo>` sensor blocks |
| Description source | **Generated once from upstream `ur_gz.urdf.xacro` into a committed URDF, then hand-edited — no generator script** |

F/T fidelity note: in `ur_macro.xacro` the `ft_frame` is at `xyz="0 0 0"` from
`wrist_3_link` (only a π roll rotation), so the simulated sensor's measurement point is
**position-coincident** with the real sensor location at the flange — there is no
lever-arm error. Only the frame orientation differs; the Stage 2 `plant_sensors` node
applies a constant rotation to republish the wrench in `tool0` coordinates (matching the
real UR driver's `ft_data` behavior).

## New files

### 1. `models/ur5e/ur_plant.urdf` (committed, hand-edited)

Generated **once** from the upstream xacro (documented command below), then edited by
hand — no generator script, no runtime code generation:

```bash
xacro "$(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/urdf/ur_gz.urdf.xacro" \
  ur_type:=ur5e \
  name:=ur \
  simulation_controllers:="$(ros2 pkg prefix ur_plant_bringup)/share/ur_plant_bringup/config/ur_plant_controllers.yaml" \
  > src/ur_plant_description/models/ur5e/ur_plant.urdf
```

(`name:=ur` is required: the top-level `<robot name="$(arg name)">` is evaluated
before the in-file default is registered, so omitting it fails with "Undefined
substitution argument name" — the same reason upstream's launch passes it.)

Hand edits applied to the committed file (values from `plant_config.yaml`, which
stays the record of values and the runtime config source for Stage 2):

- per-joint `<dynamics damping friction>` on the six joints (upstream generates zeros),
- `initial_value` params in the `<ros2_control>` block from `initial_positions`,
- the three `<gazebo>` sensor blocks below.

The file is a **committed source artifact** (not gitignored): `git diff` shows every
tuning change, and the launch spawns it directly. Caveats: upstream generates with
`force_abs_paths=true`, so mesh paths are absolute (`/opt/ros/jazzy/...`) — a distro
change means regenerating; additional robot variants are additional committed files
(`models/ur3e/ur_plant.urdf`, ...), each generated with the same command.

A sibling `model.config` turns the directory into a first-class gz-sim model,
resolvable as `model://ur5e` (the package hook exposes `models/` on
`GZ_SIM_RESOURCE_PATH`):

```xml
<?xml version="1.0"?>
<model>
  <name>ur_plant</name>
  <version>1.0</version>
  <sdf version="1.8">ur_plant.urdf</sdf>
  <description>UR5e plant with realistic joint dynamics and sensor noise</description>
</model>
```

The launch still spawns with `-file`; that the `model://` lookup resolves a
URDF-backed model is smoke-tested during verification (see below).

### 2. `config/plant_config.yaml` (record of values + Stage 2 runtime config)

```yaml
initial_positions:
  shoulder_pan_joint: 0.0
  shoulder_lift_joint: -1.57
  elbow_joint: 0.0
  wrist_1_joint: -1.57
  wrist_2_joint: 0.0
  wrist_3_joint: 0.0

joints:                 # per joint: {damping, friction, accel_limit}
  shoulder_pan_joint:   {damping: 15.0, friction: 10.0, accel_limit: 3.0}
  # ... 6 joints; damping in N·m·s/rad (viscous), friction as the URDF/SDF
  #     coefficient (start from literature ranges, calibrate — see open notes)

sensors:
  fts:
    reference: wrist_3_joint
    update_rate: 500
    frame: child          # wrist_3_link
    noise:
      force:  {stddev: 1.0, bias_stddev: 0.2}     # N,  vendor-anchored (σ_F ≈ 1 N)
      torque: {stddev: 0.05, bias_stddev: 0.01}   # Nm (σ_T ≈ 0.05 Nm)
  base_imu:
    reference: base_link
    update_rate: 500
    noise:
      angular_velocity:    {stddev: 0.0005}   # rad/s ≈ 0.03 dps [OEM chip estimate]
      linear_acceleration: {stddev: 0.01}     # m/s² ≈ 1 mg, plus dynamic bias params
  tool_accel:
    reference: tool0
    update_rate: 500
    noise:
      linear_acceleration: {stddev: 0.01}     # gyro part discarded in Stage 2

models:                 # per-model tables ur3e/ur5e/ur10e/ur16e (vendor data)
  ur5e:
    effort_limits:   [150, 150, 150, 28, 28, 28]   # Nm
    velocity_limits: [3.1416, 3.1416, 3.1416, 3.1416, 3.1416, 3.1416]  # rad/s
    fts: {force_range: 50.0, torque_range: 10.0}
  # ur3e, ur10e, ur16e: same schema

actuator:               # consumed by the Stage 2 actuator plugin
  tau_motor:            # first-order motor lag, s per joint (≈5 ms)
    default: 0.005
  torque_rate_limit: 500.0   # Nm/s, tuned empirically
```

Noise values come from the hardware research (`ur_hardware_research.md`): F/T
vendor-anchored conservative σ, IMU values from the ISM330IS/ICM42688 chip datasheets
(marked as estimates — UR publishes no IMU specs). `accel_limit` is consumed by the
controllers yaml, **not** the URDF — URDF has no standard acceleration-limit attribute.

## Sensor blocks (authored in the committed URDF)

```xml
<!-- force-torque at the last revolute joint (frame: child = wrist_3_link) -->
<gazebo reference="wrist_3_joint">
  <sensor name="tcp_fts_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>500</update_rate>
    <topic>tcp_fts_sensor</topic>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
      <force>
        <x><noise type="gaussian"><stddev>1.0</stddev><bias_mean>0</bias_mean><bias_stddev>0.2</bias_stddev></noise></x>
        <!-- y, z analogous -->
      </force>
      <torque>
        <x><noise type="gaussian"><stddev>0.05</stddev><bias_stddev>0.01</bias_stddev></noise></x>
        <!-- y, z analogous -->
      </torque>
    </force_torque>
  </sensor>
</gazebo>

<!-- 6-axis IMU at the base -->
<gazebo reference="base_link">
  <sensor name="base_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>500</update_rate>
    <topic>base_imu</topic>
    <imu>
      <angular_velocity><x><noise type="gaussian"><stddev>0.0005</stddev></noise></x>...</angular_velocity>
      <linear_acceleration><x><noise type="gaussian"><stddev>0.01</stddev><dynamic_bias_stddev>..</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></x>...</linear_acceleration>
    </imu>
  </sensor>
</gazebo>

<!-- tool accelerometer (same IMU schema; gyro discarded in Stage 2) -->
<gazebo reference="tool0"><sensor name="tool_accel" type="imu">
  <always_on>true</always_on>
  <update_rate>500</update_rate>
  <topic>tool_accel</topic>
  <imu>...</imu>
</sensor></gazebo>
```

Each sensor pins a **fully qualified** `<topic>` (`/model/ur/<name>`): gz-sim publishes
explicit sensor topics at root scope, so a bare name would land on `/base_imu` instead
of the model namespace (verified empirically — this cost a debugging round). The
`/model/ur/` prefix matches the fixed spawn name and the bridge YAML entries.

Noise paths follow the sdformat14 schemas: `angular_velocity/{x,y,z}` +
`linear_acceleration/{x,y,z}` for IMU; `force/{x,y,z}` + `torque/{x,y,z}` for
force_torque. Gaussian attributes: `mean`, `stddev`, `bias_mean`, `bias_stddev`,
`dynamic_bias_stddev`, `dynamic_bias_correlation_time`, `precision`.
Precedent for `<gazebo reference>` sensor blocks in URDF surviving the URDF→SDF
conversion: libsdformat14's `parser_urdf.cc` copies gazebo extensions verbatim
(`velodyne_description` ships exactly this idiom).

## Launch integration (`ur_plant_bringup`)

- The launch uses the committed file directly: `robot_description` is read from
  `models/<ur_type>/ur_plant.urdf` (via `FindPackageShare("ur_plant_description")`),
  and `ros_gz_sim create` spawns with `-file <path>` — replacing the upstream
  `-string` + xacro `Command` substitution. No code generation at launch time.
  (With the sibling `model.config`, including the robot in the world via a
  `model://ur5e` URI is also possible; the launch uses `-file` either way.)
- `controllers_file` flows through as the `simulation_controllers` xacro arg into the
  `gz_ros2_control-system` plugin `<parameters>` element, as upstream does.
- The robot is spawned with the **fixed name `ur`** (no `-allow_renaming`), so the gz
  sensor topics' `/model/ur/...` namespace is deterministic.
- One `parameter_bridge` node is loaded from `config/bridges.yaml` via
  `parameters=[{"config_file": ...}]` (see companion change 3), replacing upstream's
  inline CLI mapping.

## Required companion changes (same stage)

1. **`ur_plant_world/worlds/ur_plant_world.sdf`** must explicitly load
   `gz-sim-imu-system` and `gz-sim-forcetorque-system` plugins — gz-sim does not
   auto-load sensor systems.
2. **`ur_plant_bringup/config/ur_plant_controllers.yaml`** — effort-mode JTC with
   per-joint PID gains (seeded values; tuned once in Stage 2 against the complete
   plant). Example:

   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 500

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster
         # no state_publish_rate -> publishes at the full 500 Hz CM rate,
         # matching the real UR driver

       io_and_status_controller:
         type: ur_controllers/GPIOController

       speed_scaling_state_broadcaster:
         type: ur_controllers/SpeedScalingStateBroadcaster

       joint_trajectory_controller:
         type: joint_trajectory_controller/JointTrajectoryController

       scaled_joint_trajectory_controller:
         type: ur_controllers/ScaledJointTrajectoryController

       forward_velocity_controller:
         type: velocity_controllers/JointGroupVelocityController

       forward_position_controller:
         type: position_controllers/JointGroupPositionController

   scaled_joint_trajectory_controller:
     ros__parameters:
       joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                wrist_1_joint, wrist_2_joint, wrist_3_joint]
       command_interfaces: [effort]
       state_interfaces: [position, velocity]
       state_publish_rate: 100.0
       action_monitor_rate: 20.0
       allow_partial_joints_goal: false
       speed_scaling_interface_name: ""

       constraints:
         stopped_velocity_tolerance: 0.2
         goal_time: 0.0
         shoulder_pan_joint: {trajectory: 0.2, goal: 0.1}
         shoulder_lift_joint: {trajectory: 0.2, goal: 0.1}
         elbow_joint: {trajectory: 0.2, goal: 0.1}
         wrist_1_joint: {trajectory: 0.2, goal: 0.1}
         wrist_2_joint: {trajectory: 0.2, goal: 0.1}
         wrist_3_joint: {trajectory: 0.2, goal: 0.1}

       # Seeded gains; tuned once in Stage 2 against the complete plant.
       # i_clamp stays well below the vendor joint torque ratings (150 Nm
       # shoulder/elbow, 28 Nm wrist).
       gains:
         shoulder_pan_joint:  {p: 4000.0, i: 400.0, d: 50.0, i_clamp: 40.0}
         shoulder_lift_joint: {p: 4000.0, i: 400.0, d: 50.0, i_clamp: 40.0}
         elbow_joint:         {p: 3500.0, i: 350.0, d: 40.0, i_clamp: 40.0}
         wrist_1_joint:       {p: 800.0,  i: 50.0,  d: 10.0, i_clamp: 8.0}
         wrist_2_joint:       {p: 800.0,  i: 50.0,  d: 10.0, i_clamp: 8.0}
         wrist_3_joint:       {p: 500.0,  i: 30.0,  d: 5.0,  i_clamp: 5.0}

   joint_trajectory_controller:
     ros__parameters:
       # identical joints/interfaces/constraints/gains block for the plain
       # (unscaled) JTC; the scaled one is the launch default, as upstream
   ```

   Notes on limits: `state_interfaces` must stay `[position, velocity]` — JTC rejects
   `effort` there (allowed set is `{position, velocity, acceleration}`); effort still
   reaches `/raw_joint_states` via the broadcaster's state read. JTC clamps commanded
   **velocity** to the URDF limits (180 °/s, already in `ur_description`) — nothing to
   add. **Acceleration** has no URDF attribute and no JTC parameter (UR publishes no
   accel limits), so effective acceleration limiting comes from the trajectory's
   quintic-spline interpolation plus the Stage 2 actuator plugin's torque rate limit;
   `accel_limit` in `plant_config.yaml` is therefore informational for now. The
   `i_clamp` values bound integrator windup below the vendor torque ratings.
3. **Bridges** — `config/bridges.yaml` in `ur_plant_bringup`, loaded through the
   bridge node's `config_file` parameter (the standard ros_gz_bridge YAML form):

   ```yaml
   - ros_topic_name: "/clock"
     gz_topic_name: "/clock"
     ros_type_name: "rosgraph_msgs/msg/Clock"
     gz_type_name: "gz.msgs.Clock"
     direction: GZ_TO_ROS

   - ros_topic_name: "/plant/imu"
     gz_topic_name: "/model/ur/base_imu"
     ros_type_name: "sensor_msgs/msg/Imu"
     gz_type_name: "gz.msgs.IMU"
     direction: GZ_TO_ROS

   - ros_topic_name: "/plant/tool_accel"
     gz_topic_name: "/model/ur/tool_accel"
     ros_type_name: "sensor_msgs/msg/Imu"
     gz_type_name: "gz.msgs.IMU"
     direction: GZ_TO_ROS

   - ros_topic_name: "/plant/ft_sensor/wrench"
     gz_topic_name: "/model/ur/tcp_fts_sensor"
     ros_type_name: "geometry_msgs/msg/Wrench"
     gz_type_name: "gz.msgs.Wrench"
     direction: GZ_TO_ROS
   ```

   All entries `GZ_TO_ROS` (sensors only in Stage 1); the upstream inline `/clock`
   mapping is folded in. The gz-side names assume (a) the fixed spawn name `ur` and
   (b) the explicit `<topic>` per sensor pinned by the generator — so the mapping
   cannot drift. `geometry_msgs/Wrench` carries no header; the Stage 2
   `plant_sensors` node adds timestamps and the `tool0` frame transform.

## Verification

1. `check_urdf` on the committed `models/ur5e/ur_plant.urdf` passes; grep confirms
   non-zero dynamics on all 6 joints and the three sensor blocks.
2. Launch headless → `gz topic -l` shows `/model/ur/tcp_fts_sensor`,
   `/model/ur/base_imu`, `/model/ur/tool_accel`; bridged topics publish at ~500 Hz;
   `example_move.py` moves the arm; lift joints show gravity-holding effort.
   **Sanity check only — the seeded PID gains are not tuned here** (servo-emulation
   tuning happens in Stage 2 against the complete plant, once the actuator dynamics
   and friction model are in; adjust gains here only if the arm cannot hold or move).

   **Result (2026-10-31): PASSED with a known limitation.** Sensors publish at 500 Hz;
   the arm holds the upright pose under the seeded gains; `example_move.py` moves the
   arm partway before aborting on a joint-4 velocity tolerance — tracking precision is
   a Stage 2 concern (servo-emulation gains + friction calibration together; note the
   abort is partly torque-saturation-limited, so gains alone may not fix it).
3. Smoke-test the `model://ur5e` URI: a throwaway world with
   `<include><uri>model://ur5e</uri></include>` loads the plant, confirming gz-sim
   resolves the URDF-backed model.

## Open notes

- URDF/DART `friction` is a unitless coefficient, not Nm Coulomb — start from the
  literature ranges (7–12 Nm Coulomb / 9–15 N·m·s/rad viscous, converted
  empirically), calibrate in sim; the plugin-based Coulomb-Nm friction remains the
  Stage 2 fallback.
- Friction must leave torque budget for the documented max joint speed
  (`B·ω_max + C ≪ effort limit`); the literature viscous range only fits the 150 Nm
  shoulder/elbow joints, so the 28 Nm wrists get reduced seeds (see
  `plant_config.yaml`). Full friction calibration happens in Stage 2.
- The F/T wrench is expressed in the `wrist_3_link` frame; the Stage 2 sensors node
  rotates it to `tool0` (constant rotation — see fidelity note above).
- `ros_gz_bridge` maps `gz.msgs.Wrench` → `geometry_msgs/Wrench` (no header); the
  Stage 2 node adds timestamps/frames when republishing.
