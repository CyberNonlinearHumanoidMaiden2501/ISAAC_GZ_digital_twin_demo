# Stage 2 Plan: C++ Realism Layer (actuator, faults, sensors, thermal)

> Status: Step 1 (Octave reference model) COMPLETE 2026-11-25; Step 2 planned
> in sub-stages 2.1 plugin framework → 2.2 PMSM electrical/mechanical →
> 2.3 electronics + state publishing → 2.4 JTC tuning; remaining steps TBD.
> Parent plan: [`plan_realistic_ur_plant.md`](plan_realistic_ur_plant.md).
> Preceding stage: [`plan_stage1_description.md`](plan_stage1_description.md)
> — finished 2026-10-31.
> Scope: everything that turns the Stage 1 config-level plant into a physically
> realistic one — the L2 actuator cascade, mechanical/electrical fault injection,
> the sensor-fault node, the thermal model, and the single servo-tuning session.
> Stage 2 is the core of this plant: this file is the working plan for it.

## What Stage 2 delivers

| Component | Package | Content |
|---|---|---|
| L2 actuator cascade | `ur_plant_world` | `actuator_dynamics.cpp` — per-joint PMSM electrical stage (PI current loop, back-EMF, PWM transport delay, cogging ripple) + two-mass flexspline dynamics + friction in physical Nm + published per-joint actuator state topics; `l1`/`l0` config fallbacks |
| Mechanical/electrical faults | `ur_plant_world` | `fault_injection.cpp` — `/plant/mechanical_faults` service: `gearbox_wear`, `joint_jam`, `brake_drag`, `torque_derate`, `payload_misconfig`, `collision_push`, `voltage_sag`, `joint_overheat` |
| Sensor faults + noisy topics | `ur_plant_sensors` | `plant_sensors` node: `/plant/joint_states` (quantized, noisy, drifting encoders), FTS republish with zeroing service, tool accelerometer; `/plant/sensor_faults`: `encoder_fault`, `fts_zero_shift`, `imu_fault`, `comms_latency`; `SetFault.srv` + `inject_fault.py` CLI |
| Thermal model | `ur_plant_world` | first-order duty-cycle → joint temperature; 80 °C warning, 85 °C shutdown, no auto-recovery |
| Servo tuning | Step 2 | one tuning session: JTC PID against the Gazebo plant (the current-loop design already lives in the Octave reference) |
| Docs | `ur_plant_world` | README documenting every fault and its expected signature |

## Key design decisions already made

- **Actuator depth: L2** (decided 2026-11-02) — see the L0–L2 ladder in the
  parent plan, Component 5.
- **Friction lives in the plugin in physical Nm**; the unitless URDF friction is
  neutralized while the plugin is active.
- **FTS noise: vendor-conservative worst case** as default; literature model
  stays a config option.
- **Friction: the Gnad 2026 per-joint table** (obtained 2026-11-24,
  `docs/sources/literature/friction_table.xlsx`, UR5e mean of two robots);
  asymmetric Coulomb is a later refinement.
- Faults carry a `severity` field; the protective-stop / safety state machine
  stays deferred.

## Research sub-tasks (prerequisites for full parameterization)

1. Extract the flexspline stiffness curve k(φ) from the Madsen thesis figures
   (`docs/sources/literature/`).
2. Source R / L / k_t / k_e for the candidate Kollmorgen TBM2G motors
   (datasheets).
3. ~~The Gnad 2026 per-joint friction table~~ — obtained 2026-11-24:
   `docs/sources/literature/friction_table.xlsx` (UR5e mean of two robots).

## Acceptance criteria (from the parent plan)

- `example_move.py` completes without tolerance aborts — the Stage 1 abort was
  partly torque-saturation-limited, so friction calibration is part of the fix,
  not just gain tuning.
- Step responses look like a real servo: finite rise time, slight overshoot,
  ~50–200 ms settling at the 500 Hz loop.
- Every fault is injectable at runtime and shows its documented signature
  (Stage 3 formalizes these as automated tests).

## Step-by-step breakdown

### Step 1: Octave reference model — `octave/actuator/`

The actuator is modeled and tested in **Octave first** (Octave 8.4 + control
package; MATLAB unavailable), before any C++ — the plugin port in Step 2 is
validated against this reference. Fixed-step integration at the switching
period (10 kHz) mirrors the future plugin's sub-step structure, so the Octave
loop and the C++ loop are the same algorithm. Progression: 1.1 electrical
stage → 1.2 mechanical stage → 1.3 electronics layer + integrated plant
verification. Position-loop tuning is deliberately NOT part of Step 1 — the
real 500 Hz JTC runs in the sim, so the tuning session happens in Step 2
against Gazebo, not against a recreated controller.

**Design decision (2026-11-18): model the PMSM in the d-q frame** — the FOC frame
every real servo drive runs in, so the plant becomes a literal emulation of the
drive's internal loop. Rationale:

- **Parameters map 1:1 to datasheets**: R, L_d ≈ L_q, λ_pm (or Kt/Ke), pole pairs
  p — exactly what the Kollmorgen TBM2G datasheets publish; no derived guesses.
- **Faults become physics, not hacks**: `voltage_sag` = lower V_dc → the voltage
  ceiling shrinks (C296A44/C50A51); overcurrent = trip on |i| (C59A); derating =
  lower the current limit; regen clamp = V_dc ≤ 55 V (C56A).
- **Flux weakening only exists in d-q**: if the candidate motors cannot reach
  rated speed on the 48 V bus, the fix is i_d < 0 — inexpressible in a scalar
  torque-source model.
- **Cheap**: three extra states per joint (i_d, i_q, θ_e = p·θ_m), and the rotor
  angle already exists for the two-mass model.

**Control chain:**

```
ROS (500 Hz)               plugin (sub-steps at 10 kHz)                      DART (1 kHz)
JTC PID → τ_j* ──→ i_q* = (τ_j*/N + τ̂_fric,motor)/Kt
                     → d-q PI current loops (i_d* = 0) + decoupling
                       (+ω_e·L_q·i_q , −ω_e·L_d·i_d − ω_e·λ_pm)
                     → transport delay 1.5·T_sw → |V| ≤ V_dc/√3
                     → PMSM ODE → τ_em = 1.5·p·λ_pm·i_q
                     → rotor EOM → flexspline τ_fs = k(φ)·φ + d·φ̇ → joint torque
```

**Ideal model equations** (surface PMSM, L_d = L_q = L):

```
V_d = R·i_d + L·di_d/dt − ω_e·L·i_q
V_q = R·i_q + L·di_q/dt + ω_e·L·i_d + ω_e·λ_pm
τ_em = 1.5·p·λ_pm·i_q = Kt·i_q          (i_d* = 0)
J_m·dω_m/dt = τ_em − τ_fric,motor − τ_fs/N
τ_fs = k(φ)·φ + d·φ̇ ,   φ = θ_m/N − θ_j
```

**The two-mass model — why the rotor and the link are two separate masses.** A
rigid-joint model assumes θ_m = N·θ_j exactly. A strain-wave gear violates
that: the flexspline is a thin-walled cup that twists elastically under load,
so rotor and link are two inertias coupled by a torsional spring with windup
φ = θ_m/N − θ_j and transmitted torque τ_fs = k(φ)·φ + d·φ̇ — the standard
flexible-joint model (Spong), the same one the Madsen thesis identifies on the
UR5e.

- **Steady windup:** φ = τ/k — the gear deflects proportionally to transmitted
  torque; the shoulder-lift joint holding gravity (~50–100 Nm) sits at
  φ ≈ 0.3–0.6° (at k = 1e4 Nm/rad). Every torque reversal winds the spring
  the other way.
- **Torsional resonance:** the reflected rotor inertia J_m·N² ≈ 0.65 kg·m² is
  comparable to the link inertia, so the pair oscillates. In the transfer from
  motor torque to MOTOR position there is an anti-resonance (zero)
  ω_ar = √(k/J_link) ≈ 19.5 Hz: at that frequency the link + spring act as a
  vibration absorber — the rotor stands still while the LINK oscillates
  (visible on the joint encoder, absent from the motor encoder). The
  resonance ω_res = √(k·(J_m·N² + J_link)/(J_m·N²·J_link)) ≈ 31 Hz appears in
  both responses (J_link = 1.0, k = 1.5e4 seeds — k pending the Madsen figure
  extraction). This structural mode sits inside the 500 Hz sensor bandwidth
  and the JTC PID bandwidth: residual vibration after moves, and a hard
  ceiling on achievable servo bandwidth — the research doc's
  "servo bandwidth: not published [NF]" emerges from this physics instead of
  being asserted. (Corrected 2026-11-24: the anti-resonance is a zero of the
  MOTOR response, not the link response.)
- **Fault signature:** the motor-vs-joint encoder divergence θ_m/N − θ_j = φ is
  the basis of C46A, C75/C265 and gearbox-wear detection — a degrading
  flexspline lowers k, so the resonance drops and windup grows.
- **Scope:** stiffening k(φ) + linear damping d first; hysteresis is out of the
  first cut.
- **Implementation:** the rotor ODE is integrated in the plugin (10 kHz
  sub-steps); τ_fs is applied to DART as an explicit joint torque at 1 kHz, so
  the spring stiffness never enters DART's constraint solver (no stiff
  problem). φ̇ = ω_m/N − ω_j needs only velocities from DART.

**What SVPWM does to the circuit.** The motor winding is an R-L filter: it sees
the one-period average of the switched voltage plus a switched ripple.
State-space averaging replaces the switched circuit with its one-period average
and is exact to first order — this is how real drive loops are designed. The
effects, ranked by plant relevance (V_dc = 48 V, f_sw = 10 kHz default — config):

| Effect | Mechanism | Magnitude | Plant relevance |
|---|---|---|---|
| Computational + ZOH delay | voltage applied in period k+1 is computed from samples in period k | 1–1.5·T_sw ≈ 100–150 µs | **Yes** — this is the servo bandwidth the research could not find published; it caps the current-loop crossover at ~f_sw/10–20 ≈ 0.5–1 kHz. Modeled as a transport delay |
| Voltage ceiling | SVPWM linear region caps phase voltage at V_dc/√3 | hard limit ≈ 27.7 V peak | **Yes** — decides whether flux weakening is needed (open check below) |
| Current ripple | switched voltage minus its average integrates on L | Δi_pp ≈ V_dc/(8·L·f_sw) ≈ 2–4 A on ~10 A_rms rated (verified L) | Signal only — averages to zero torque (rotor inertia filters f_sw). Added analytically to the measured current, never integrated |
| Dead-time | ~0.5–2 µs per period with both switches off (shoot-through guard) | constant voltage error ≈ V_dc·(t_dt/T_sw) ≈ 0.5 V, sign-opposed to current | Minor — 6th-harmonic distortion + zero-crossing clamp at low speed. Constant offset term `V_dt·sign(i)`; skipped in v1 |
| Device drops | IGBT/MOSFET on-drop + diode drop | 1–2 V offset | Folded into the dead-time term |
| Overmodulation / six-step | operating past the ceiling | buys ~10 % voltage, adds 5th/7th harmonics | Deferred |

Summary: the switching disappears into the average; what survives is (1) a
~150 µs delay that sets the torque-loop bandwidth, (2) a hard voltage ceiling
that bites at speed, (3) ripple on the current signal, (4) a small dead-time
offset. Nothing exotic to model.

**Design notes**

- The JTC's effort interface commands **joint-side** torque; the plugin reflects
  it through the gearbox (`τ_j*/N`) plus motor-friction feedforward to get the
  motor torque reference — the JTC integral action then handles residual
  friction (standard flexible-joint control partition).
- The plugin sub-steps the electrical ODE at 10 kHz internally and exchanges
  torque with DART at the world's 1 kHz; DART only ever sees joint-side torque.
- **Flux-weakening check (datasheet-verified 2026-11-23):** at max joint speed
  180 °/s the motor spins ≈ 101·3.14 ≈ 317 rad/s. With the verified Kollmorgen
  TBM2G numbers, the 06826 A/C windings' back-EMF (74 / 37 V) exceeds the
  V_dc/√3 ≈ 27.7 V ceiling — only winding D fits (21.4 V, 1.30× margin). The
  05008 A fits at 1.08× (no load headroom); C is the default (2.15×).
  Defaults: J1-3 → 06826-D, wrist → 05008-C — no field weakening needed with
  these; if the Robotica paper's winding turns out to be A/C, i_d < 0 becomes
  part of the FOC (the reason d-q was chosen).

**Build order** (each layer config-togglable in `params.m`; the
`actuator_model: l1` / `l0` fallbacks remain available throughout):

- **1.1 Electrical stage (start here):** `params.m` (per-joint [OEM] Kollmorgen
  TBM2G values + a plant-truth vs controller-belief split with
  tolerance-bounded identification errors), `pmsm.m` (d-q ODE, truth only),
  `foc_loop.m` (PI current loops + decoupling + back-EMF feedforward computed
  from the *belief*, one-sample compute delay, V_dc/√3 ceiling),
  `test_electrical.m` — validates τ_e = L/R against the research values
  (0.9 ms wrist / 1.3 ms J1–3), τ_em = Kt·i_q, back-EMF at speed (coupled
  i_d–i_q steady state), the current-loop crossover *emerging* from the PI +
  delay (design point f_sw/20 = 500 Hz; typical ID errors R +3 % / L +8 % /
  Kt +3 % degrade the 2.5 % overshoot to 4.6 % and shift the achieved
  crossover to 540 Hz; tolerance-corner worst case R +10 % / L −20 % /
  Kt +10 % → 5.3 % overshoot, 400 Hz, −9.1 % torque gain — documented in
  `params.m` as a preset), and the flux-weakening check.
  **Result (2026-11-18): PASSED** — τ_e measured 1.30/0.91 ms, i_ss = V/R,
  τ_em = Kt·i_q, coupled back-EMF steady state matched analytically, voltage
  ceiling simulated = analytic (346 rad/s).
  **Result (2026-11-23): parameters re-sourced and verified** against the
  Kollmorgen datasheets (τ_e cross-checked 1.34 / 0.93 ms; all windings
  consistent); winding defaults 06826-D / 05008-C; the FOC now runs on
  identification estimates, never on true plant values.
- **1.2 Mechanical stage:** `params_mech.m` (Gnad per-joint friction +
  flexspline seeds), `two_mass.m` (rotor EOM + stiffening flexspline k(φ)),
  `test_mechanical.m` — steady windup under gravity load (stiffening visible:
  φ/φ_linear = 0.86 at 50 Nm → 0.70 at 100 Nm), the anti-resonance/resonance
  pair (19.5 / 31.0 Hz with the k0 = 1.5e4 seed; formula = control-pkg =
  kick-test FFT), friction terminal velocity (exact), motor-vs-joint encoder
  divergence. **Result (2026-11-24): PASSED.**
- **1.3 Electronics layer + integrated plant verification:** the 1c/1d
  content (required, not optional) — `inverter.m`: power stage between
  `foc_loop` and `pmsm` — dead-time offset mapped to d-q via the current
  angle (≈ V_dc·t_dt/T_sw ≈ 0.5–1 V, sign-opposed to phase current), device
  drops folded in, circular clamp tracking the actual bus; plus a one-state
  bus model C·dV_dc/dt = (P_src − P_motor)/V_dc with regen clamp at 55 V
  (C56A) — the piece that makes `voltage_sag` and overcurrent faults
  physical. `sense_current.m`: measurement path only — the drive's
  measurement is the midpoint-sampled AVERAGE (real drives sample at the PWM
  midpoint, rejecting the f_sw ripple), so the default is average + 12-bit
  ADC quantization + bias; the ripple's analytic bound
  Δi_pp ≈ V_dc/(8·L·f_sw) is available as an optional statistics-correct
  injection — feeding the virtual motor-current stream the twin consumes (a
  real 500 Hz UR `tool_data` stream). Cogging Fourier terms and fault hooks
  fold in here.
  Then the FULL reference model is wired end-to-end
  (`foc_loop` → `inverter` → `pmsm` → `two_mass` + Gnad friction) and
  verified open-loop: torque response through the gear, back-EMF at speed,
  resonance and windup intact, effort realism. Deliberately excluded
  (unchanged): switching-level integration (rotor inertia filters f_sw —
  torque sees only the period average), semiconductor transient/loss detail
  (thermal is one-way), hexagon overmodulation (config option only).
  Validation: ripple matches the analytic bound with unchanged average
  torque; dead-time 6th-harmonic at the electrical frequency; bus regen →
  55 V clamp; sag → ceiling drop → torque falloff at speed. Completion of
  1.3 = the Octave reference model is FINAL — the validation oracle for the
  plugin port. **Result (2026-11-25): PASSED** — measurement stats exact
  (bias 0.0203 vs set 0.02, std = ADC lsb/√12, ripple std 0.869 vs analytic
  0.888); dead-time 6th harmonic 0.020 V vs 0.002 V with t_dt = 0 at
  6·f_e = 955 Hz; regen clamps the bus at 55.00 V (diode-isolated supply);
  sag (Vsrc = 40 V) at ω_m = 330 drops i_q 4.99 → 0.61 A (88 % falloff — the
  point sits at the no-FW ceiling, where sag + dead-time leave almost no
  voltage); integrated chain: terminal ω_j 1.5869 rad/s = expected exactly,
  steady τ_fs = pC + pv·ω_j, windup 2.6 mrad. **Step 1 complete.**

Acceptance for step 1: the complete Octave reference (electrical +
mechanical + electronics) passes all stage tests — the validated oracle the
plugin is ported against. Position-loop (JTC PID) tuning is deliberately not
done in Octave.

### Step 2: Plugin development + servo tuning — `actuator_dynamics.cpp`

Three sub-stages; the Octave reference (Step 1) is the validation oracle at
every boundary. The plugin lives in `ur_plant_world` (namespace
`ur_plant_world`, source `src/ur_plant_world/src/`, header
`include/ur_plant_world/`), loaded from `worlds/ur_plant_world.sdf`.

#### 2.1 Plugin framework (empty shell)

- Standard gz-sim system plugin skeleton: `ISystemConfigure`/`ISystemPreUpdate`
  + `GZ_ADD_PLUGIN`, registered in `worlds/ur_plant_world.sdf`.
- Configure: read SDF params (config-file path, `actuator_model`), resolve
  the model's joints. PreUpdate: for each joint, read the `JointForceCmd`
  component (set by gz_ros2_control's GazeboSimSystem), log it (throttled),
  pass it through unchanged (`l0` behavior).
- Build: CMakeLists gets the plugin target
  (`ur_plant_world_actuator_dynamics` shared lib, gz-sim cmake helpers);
  the template's commented plugin target is finally used.
- Acceptance: clean colcon build; headless launch; the plugin loads and
  logs the joints; `example_move.py` behaves exactly as Stage 1 (passthrough
  = no actuator filtering yet).
  **Result (2026-11-25): PASSED** — the plugin loads, resolves the six
  actuated joints lazily (the robot is spawned after world Configure, so
  PreUpdate retries until it appears; the fixed `base_joint` is filtered by
  joint type), logs the gz_ros2_control effort commands, and
  `example_move.py` aborts with the identical Stage-1
  `GOAL_TOLERANCE_VIOLATED` signature — passthrough changes nothing.

#### 2.2 PMSM electrical + mechanical (motor side + flexspline)

- Port `pmsm.m` (i_d, i_q), the FOC current loop (`foc_loop.m`: PI +
  decoupling from the BELIEF, one-sample compute delay, circular clamp), and
  the motor side of `two_mass.m` (θ_m, ω_m, flexspline τ_fs). The LINK side
  is DART's: link inertia/gravity come from Gazebo, NOT the plugin — the
  plugin applies τ_j = τ_fs − (pC·tanh(ω_j/ε) + pv·ω_j) (Gnad joint
  friction) to the joint and reads θ_j, ω_j back from DART.
- Command path: `JointForceCmd` (joint effort) → τ_m* = τ_j*/N + motor
  friction feedforward → i_q* = τ_m*/Kt̂ (the belief, as in the reference).
- Sub-step loop: 10 kHz internal integration (RK4, as in Octave), torque
  exchange with DART at the world's 1 kHz. Per-joint params
  (R/L/λ/p/N/Jm/k0/k2/d/pC/pv/ε, belief errors) load from
  `plant_config.yaml` — the placeholder `actuator:` block is replaced by the
  real per-joint L2 table (single source of truth; yaml-cpp dependency).
- Acceptance: for a constant effort command, the terminal joint velocity in
  Gazebo (from `/joint_states`) matches the Octave integrated-test
  prediction (N·Kt·i_q − pC)/pv; windup/friction visible in tracking; the
  arm still holds and moves under the JTC with the seeded PID.
  **Result (2026-11-25): PASSED (with one deferral).** The plugin runs the
  cascade correctly — during `example_move.py` the logged chain is exact
  (cmd −45.8 Nm → i_q −4.37 A = τ_j*/N/Kt̂ to 3 digits; windup −2.6 mrad
  under ~46 Nm matches the Octave stiffening scale) — and an l0-vs-l2 A/B
  run proves the physics consumes the resolved torque (the JTC's commanded
  effort range shifts from −41…+8 Nm to −56…+20 Nm when the actuator is in
  the loop). The arm holds the start pose (~zero effort at the near-stable
  upright pose) and `example_move.py` moves it, aborting with the expected
  tolerance signature (PID tuning is 2.4). Two fixes recorded: the plugin is
  now model-attached and declared after `gz_ros2_control` (PreUpdate
  ordering — the world-level plugin read stale commands), and the
  SDF/URDF `actuator_model` takes precedence over the yaml `model:` field
  (the yaml value silently overrode the A/B control run). The explicit
  constant-effort terminal-velocity check is deferred to 2.3, where the
  published state topics make the plugin-vs-Octave comparison mechanical.

#### 2.3 Electronics + state observability

- Port `inverter.m` (dead-time in d-q, diode-isolated bus state, 55 V regen
  clamp) and `sense_current.m` (midpoint-average + ADC + bias) into the
  plugin chain.
- **State publishing:** per-joint gz-transport topics at 1 kHz under
  `/plant/actuator/<joint>/`: motor current (i_d, i_q), motor velocity ω_m,
  bus voltage V_dc, flexspline windup φ, torques τ_em/τ_fs. New entries in
  `bridges.yaml` map them to ROS topics for the twin and for validation.
- Fault hooks reserved: Vsrc / current-limit fields exposed in config so
  the later `fault_injection` plugin can drive them.
- Acceptance: topics publish at 1 kHz; **plugin ≡ Octave validation**: the
  same effort input sequence → published ω_m/τ_fs/i_q traces match the
  Octave reference (Step 1's integrated test) within numerical tolerance;
  the regen and sag scenarios re-run in Gazebo with matching outcomes.
  `actuator_model: l1` (two-mass, no electrical stage) and `l0`
  (passthrough) remain selectable throughout.

#### 2.4 JTC servo tuning

Tune the JTC PID against the Gazebo plant — the real 500 Hz JTC runs in the
sim, so this is where the servo-emulation tuning session lives (not
recreated in Octave). Acceptance (feeds the overall Stage 2 acceptance):
step responses show finite rise time + slight overshoot + ~50–200 ms
settling at the 500 Hz loop, and `example_move.py` completes without
tolerance aborts — the Stage 1 abort was partly torque-saturation-limited,
so friction calibration is part of the fix, not just gain tuning.

### Step 3: TBD — fault injection → sensors → thermal → README (ordering TBD)
