# Stage 2 Plan: C++ Realism Layer (actuator, faults, sensors, thermal)

> Status: DRAFT — Step 1 (actuator modeling) designed 2026-11-18; remaining steps
> to be added as the implementation order is decided.
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
| L2 actuator cascade | `ur_plant_world` | `actuator_dynamics.cpp` — per-joint PMSM electrical stage (PI current loop, back-EMF, PWM transport delay, cogging ripple) + two-mass flexspline dynamics + friction in physical Nm; `l1`/`l0` config fallbacks |
| Mechanical/electrical faults | `ur_plant_world` | `fault_injection.cpp` — `/plant/mechanical_faults` service: `gearbox_wear`, `joint_jam`, `brake_drag`, `torque_derate`, `payload_misconfig`, `collision_push`, `voltage_sag`, `joint_overheat` |
| Sensor faults + noisy topics | `ur_plant_sensors` | `plant_sensors` node: `/plant/joint_states` (quantized, noisy, drifting encoders), FTS republish with zeroing service, tool accelerometer; `/plant/sensor_faults`: `encoder_fault`, `fts_zero_shift`, `imu_fault`, `comms_latency`; `SetFault.srv` + `inject_fault.py` CLI |
| Thermal model | `ur_plant_world` | first-order duty-cycle → joint temperature; 80 °C warning, 85 °C shutdown, no auto-recovery |
| Servo tuning | all | one tuning session: JTC PID + current loop against the complete plant |
| Docs | `ur_plant_world` | README documenting every fault and its expected signature |

## Key design decisions already made

- **Actuator depth: L2** (decided 2026-11-02) — see the L0–L2 ladder in the
  parent plan, Component 5.
- **Friction lives in the plugin in physical Nm**; the unitless URDF friction is
  neutralized while the plugin is active.
- **FTS noise: vendor-conservative worst case** as default; literature model
  stays a config option.
- **Friction split: literature ranges + in-sim calibration** until the Gnad 2026
  per-joint table is obtained.
- Faults carry a `severity` field; the protective-stop / safety state machine
  stays deferred.

## Research sub-tasks (prerequisites for full parameterization)

1. Extract the flexspline stiffness curve k(φ) from the Madsen thesis figures
   (`docs/sources/literature/`).
2. Source R / L / k_t / k_e for the candidate Kollmorgen TBM2G motors
   (datasheets).
3. (If obtained) the Gnad 2026 per-joint friction table.

## Acceptance criteria (from the parent plan)

- `example_move.py` completes without tolerance aborts — the Stage 1 abort was
  partly torque-saturation-limited, so friction calibration is part of the fix,
  not just gain tuning.
- Step responses look like a real servo: finite rise time, slight overshoot,
  ~50–200 ms settling at the 500 Hz loop.
- Every fault is injectable at runtime and shows its documented signature
  (Stage 3 formalizes these as automated tests).

## Step-by-step breakdown

### Step 1: Actuator modeling — `actuator_dynamics.cpp` (gz-sim system plugin)

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
  comparable to the link inertia, so the pair oscillates. From motor torque to
  link position there is an anti-resonance ω_ar = √(k/J_link) ≈ 16 Hz (rotor
  oscillates while the link stands nearly still — visible on the motor encoder
  only) and a resonance ω_res = √(k·(J_m·N² + J_link)/(J_m·N²·J_link)) ≈ 25 Hz
  (J_link = 1.0, k = 1e4). This 15–25 Hz structural mode sits inside the
  500 Hz sensor bandwidth and the JTC PID bandwidth: residual vibration after
  moves, and a hard ceiling on achievable servo bandwidth — the research doc's
  "servo bandwidth: not published [NF]" emerges from this physics instead of
  being asserted.
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
| Current ripple | switched voltage minus its average integrates on L | Δi_pp ≈ V_dc/(8·L·f_sw) ≈ 0.5–1 A on ~5 A rated | Signal only — averages to zero torque (rotor inertia filters f_sw). Added analytically to the measured current, never integrated |
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
- **Open check (do during 1a):** at max joint speed 180 °/s the motor spins
  ≈ 101·3.14 ≈ 317 rad/s; back-EMF k_e·ω_m plus the R·i and ω_e·L·i drops must
  fit under V_dc/√3 ≈ 27.7 V. If not (pending the datasheet Ke), add flux
  weakening (i_d < 0) to 1b — the reason d-q was chosen.

**Build order** (each layer config-togglable in `plant_config.yaml`; the
`actuator_model: l1` / `l0` fallbacks remain available throughout):

1. **1a — ideal model:** d-q ODE with an ideal voltage source (no delay, no
   limit, no ripple). Validates τ_em = Kt·i_q and the electrical time constant
   L/R against the research values (0.9 ms wrist / 1.3 ms J1–3).
2. **1b — FOC layer:** the two PI current loops + decoupling, 1.5·T_sw transport
   delay, V_dc/√3 ceiling. The ~1 kHz current-loop bandwidth *emerges* here
   instead of being assumed — the point of the whole L2 decision.
3. **1c — signal/ripple layer:** analytic PWM ripple on the measured current,
   dead-time offset, cogging Fourier terms on torque (mechanical, but belongs
   in this layer).
4. **1d — fault hooks:** V_dc sag + regen clamp, current limit (derate,
   overcurrent trip), R-drift hook reserved for the thermal model.

Acceptance for step 1 (feeds the overall Stage 2 acceptance): step responses
show finite rise time + slight overshoot + ~50–200 ms settling at the 500 Hz
loop, and `example_move.py` completes without tolerance aborts once the PID is
tuned against the complete plant.

### Step 2: TBD

### Step 3: TBD
