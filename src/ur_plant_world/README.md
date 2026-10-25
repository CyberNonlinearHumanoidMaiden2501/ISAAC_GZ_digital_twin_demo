# ur_plant_world

Gazebo Harmonic worlds and gz-sim system plugins for the UR plant simulation
(see `docs/plan_realistic_ur_plant.md` in the workspace root).

- `worlds/ur_plant_world.sdf` — pinned DART physics (1 ms step, RTF 1.0, 1000 Hz
  update rate) — added in Stage 1.
- `actuator_dynamics` — first-order motor lag, torque rate limit, saturation —
  added in Stage 2.
- `fault_injection` — mechanical/electrical faults with a ROS service — added in
  Stage 2.

Environment hooks set `GZ_SIM_RESOURCE_PATH` (worlds) and `GZ_SIM_SYSTEM_PLUGIN_PATH`
(plugin lib) when the workspace is sourced.
