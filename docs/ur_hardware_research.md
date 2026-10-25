# UR Hardware Reality: Documented Parameters for the Gazebo Plant Model

> Research compiled 2026-10-25. Primary model of interest: **UR5e** (sim default).
> Legend: **[V]** = vendor-documented (Universal Robots docs/datasheets/error handbooks),
> **[L]** = literature-measured (papers/theses/teardowns), **[OEM]** = component supplier,
> **[NF]** = not found anywhere public — must be estimated.
> Sources were downloaded and read in full where possible (PDFs of UR5e user manual v5.8,
> e-Series service manual v1.1.6, current e-series datasheets, UR error-code handbooks,
> official RTDE guide, `Universal_Robots_ROS2_Driver` repo).

---

## 1. Actuators

### 1.1 Documented limits per model [V]

| Parameter | UR3e | UR5e | UR10e/UR12e | UR16e |
|---|---|---|---|---|
| Joint speed (deg/s) | 180/180/180/**360/360/360** | 180 ×6 | **120/120/180**/180/180/180 | 120/120/180/180/180/180 |
| Joint torque (Nm) | 54/54/28/9/9/9 | **150/150/150/28/28/28** | 330/330/150/54/54/54 | 330/330/150/54/54/54 |
| Payload (kg) | 3 | 5 | 12.5 (since May 2021) | 16 |
| Reach (mm) | 500 | 850 | 1300 | 900 |
| Repeatability (ISO 9283) | ±0.03 mm | ±0.03 mm | ±0.05 mm | ±0.05 mm |
| Arm weight (kg) | 11.2 | 20.7 | 33.5 | 33.1 |
| Typical power (W) | ~100 | ~250 (570 max avg) | ~350 | ~350 (585 max) |
| TCP speed | — | ≈1 m/s | up to 4 m/s (ur12e) | — |

- Torque limits come from UR's support article ["Max. joint torques"](https://www.universal-robots.com/articles/ur-articles/max-joint-torques/): 5 joint sizes (9 / 28 / 54 / 150 / 330 Nm). Identical to the UR-provided ROS data in `/opt/ros/jazzy/share/ur_description/config/<model>/joint_limits.yaml`.
- **Continuous vs peak torque rating: [NF]** — UR publishes only the single per-joint max torque. Treat it as the saturation limit.
- Speed sources: UR5e user manual v5.8 p. I-74; [UR3e tech details PDF](https://www.universal-robots.com/media/1802780/ur3e-32528_ur_technical_details_.pdf); [UR10e tech specs PDF](https://www.universal-robots.com/media/1802458/ur10e-tech-specs-eng.pdf). Note: the ROS `joint_limits.yaml` halves the UR5e elbow position range to ±180° only as a MoveIt planning aid — physically it is ±360°.

### 1.2 Servo / motor internals

| Parameter | Value | Status |
|---|---|---|
| Control loop frequency | **500 Hz** | [V] (user manual; RTDE guide) |
| Gearbox | Harmonic Drive strain-wave, **ratio ≈ 100–101:1** all joints | [L] (Robotica 2024 paper: 100; Gnad et al. IEEE Access 2026: 101 from UR config files) |
| Motor type | Kollmorgen frameless brushless AC servos (candidates TBM2G-05008 wrist / TBM2G-06826 joints 1–3) | [L] (Robotica 2024; UR confirms nothing publicly) |
| Motor-side inertia (kg·m²) | J1–3: 6.4e-5, J4–6: 1.2e-5 | [L] (Robotica 2024, Table IV) |
| Motor electrical time constant τ_e = L/R | ~0.9 ms (wrist) / ~1.3 ms (J1–3) | [L] (derived from candidate-motor datasheets) |
| **Servo bandwidth (−3 dB)** | **not published, no academic measurement found** | **[NF]** — estimate |
| Joint acceleration limits | **not published** | [NF] (stated explicitly in ROS `joint_limits.yaml` comments) |

Total torque-response lag ≈ electrical τ (~1–3 ms) + 2 ms discrete loop at 500 Hz → **first-order τ ≈ 3–5 ms** is a defensible plant model.

### 1.3 Joint friction [L]

- UR5e measured joint-side friction: **Coulomb 7–12 Nm, viscous 9–15 N·m·s/rad** (ranges across the 6 joints) — [Gnad, Gattringer, Müller et al., IEEE Access 14:138706, 2026](https://doi.org/10.1109/ACCESS.2026.3726682). Exact per-joint table is paywalled (verify before using precise values).
- Model form: **asymmetric** Coulomb (direction-dependent — different when power flows forward vs backward through the strain-wave gear) + viscous; smoothing ε=1e-3 — Robotica 2024 (["Electromechanical modeling and identification of the UR5 e-Series robot"](https://www.cambridge.org/core/journals/robotica/article/electromechanical-modeling-and-identification-of-the-ur5-eseries-robot/1AE5BAE866D904F6C79B4C159BEA2B45F)).
- Temperature dependence: viscous friction drops up to **35%** from "cold" (21–25°C) to "warm" (30–37°C) robot — Kovincic et al. 2019 (UR5 CB3).

### 1.4 Brakes [V]

- **All six joints have a brake**: electromagnetic brake solenoid + mechanical brake pin + a **friction clutch** that lets the joint slip under forced back-driving (e-Series Service Manual v1.1.6).
- Forced back-driving limited to ≤160° per joint; brakes are not designed for manual release; Freedrive/Backdrive releases brakes only in joints where pressure is applied.
- **Brake holding torque / deceleration rating: [NF]** — no vendor number. (The "50 ms / 40 mm" stopping figures in the manual are user-configurable *safety-function limits*, not actuator constants.)

### 1.5 Temperature [V]

| Condition | Value |
|---|---|
| Ambient operating range | 0–50 °C |
| Performance derating | above 35 °C ambient, robot may run at reduced speed |
| Joint internal temp — warning | **80 °C** (error C62A1) |
| Joint internal temp — shutdown | **85 °C** (error C62A11) |
| Startup temperature alignment | must reach 45 °C within 30 min (C190A11/12) |

---

## 2. Sensors

### 2.1 Joint encoders

- **Dual encoders per joint** [V+L]: motor-side **incremental optical** encoder (commutation/velocity loop) + output-side **single-turn absolute magnetic** encoder (readhead + ring) on the reducer output. Vendor error handbooks distinguish "Motor Encoder" (C263) from "Joint Encoder" (C265); Madsen PhD thesis confirms the architecture ([pure.au.dk PDF](https://pure.au.dk/ws/files/197603831/Phd_Thesis_Emil_Madsen.pdf)).
- Output encoder series: **RLS AksIM** absolute magnetic rotary (Renishaw associate), series spec **up to 20-bit**, accuracy up to ±0.1° — [Renishaw case study](https://www.renishaw.com/nl/aksim-supports-universal-robots-for-smart-factory-automation--40389). **The UR-configured bit depth is not published [NF].**
- Motor encoder: 1080 CPR optical [L] (PoliTo thesis, [webthesis.biblio.polito.it/16939/1/tesi.pdf](https://webthesis.biblio.polito.it/16939/1/tesi.pdf)).
- Derived quantization (estimate): 1080 CPR ×4 quadrature / 101:1 ≈ 1.4e-5 rad at joint output; 19–20-bit output encoder ≈ 0.6–1.2e-5 rad/LSB. → **quantization step ≈ 1e-5 rad is a defensible plant value.**
- **Encoder noise stddev: [NF]** — no published measurement found. Estimate σ ≈ 0.5–1 LSB ≈ 0.5–1e-5 rad.

### 2.2 TCP force-torque sensor (e-series integrated FTS) [V]

Current official datasheets (`universal-robots.com/manuals/latest/en/datasheets/<model>/`):

| Model | Force range | Torque range | Precision (F/T) | Accuracy (F/T) |
|---|---|---|---|---|
| UR3e | ±30 N | ±10 Nm | ±2.0 N / ±0.1 Nm | ±3.5 N / ±0.1 Nm |
| **UR5e** | **±50 N** | **±10 Nm** | **±3.5 N / ±0.2 Nm** | **±4.0 N / ±0.3 Nm** |
| UR10e | ±100 N | ±10 Nm | ±5.0 N / ±0.2 Nm | ±5.5 N / ±0.5 Nm |
| UR16e | ±160 N | ±10 Nm | ±5.0 N / ±0.2 Nm | ±5.5 N / ±0.5 Nm |

- Update rate: **500 Hz** (RTDE data, "as high as the real-time control loop frequency" — [RTDE guide](https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/)).
- The sensor is **relative**: requires zeroing (`~/zero_ftsensor` in the UR driver; error family C450). Plant must emulate zero-offset + drift.
- Noise spectrum: not vendor-published. One literature model (UoP thesis, full text unverified): 1st-order LP f_c=100 Hz + quantization 0.2 N + Gaussian σ=0.3 N.
- Discrepancy note: older UR docs listed UR5e FTS at 30 N / 2.5 N resolution; the current datasheet (50 N / 3.5 N) is authoritative.

### 2.3 Internal IMUs — correction to the earlier plan note

Stock e-series arms are **not IMU-free**:
- Base: **two 6-axis MEMS IMUs on the Base Filter Board** (STM ISM330IS + TDK ICM42688) [V, error code C353].
- Tool flange: **accelerometer on the Tool PCBA** [V, error code C363]; exposed via RTDE as `actual_tool_accelerometer` (SW ≥5.4).
- Newer generation: SCB IMU (C453).
- **No published specs** (range/noise/bandwidth) — they are internal safety/state-monitoring devices, not user-facing sensors [NF].

Implication: still correct to omit an IMU from the plant's *exposed* sensor suite (nothing vendor-published to ground it), but the claim "real UR has no IMU" was wrong — it has internal ones. A tool-accelerometer topic could be added later if wanted.

### 2.4 Driver publication rates [V]

| Stream | Rate |
|---|---|
| `joint_states` (JSB, no `state_publish_rate` → every CM update) | **500 Hz** |
| TCP wrench (`ft_data`, `force_torque_sensor_broadcaster`, frame `tool0_controller`) | **500 Hz** |
| Tool data (`tool_data`: current, voltage, temperature) | 500 Hz |
| JTC internal state topic (`state_publish_rate: 100.0`) | 100 Hz |

### 2.5 Joint temperature + tool I/O [V]

- Each joint has a temperature sensor (RTDE `joint_temperatures`); thresholds 80/85 °C; >50 °C logged as warning by the joint safety processor.
- Tool I/O: 12/24 V supply, digital output typical **600 mA** (peak 1000 mA), tool supply 1–1.5 A typ, 2× 12-bit analog inputs. (The widely quoted "600 mA" is the CB3 tool-supply limit / e-series digital-output typical.)

---

## 3. Faults & protective behavior

### 3.1 Stop categories [V] (IEC 60204-1, UR5e manual §4.5)

| Category | Behavior |
|---|---|
| **0** | Immediate removal of drive power; brakes engage |
| **1** | Orderly controlled deceleration with power on; power removed at monitored standstill; brakes engaged |
| **2** | Controlled stop along the trajectory; **power stays on** after stopping |

Safety-system trigger → reaction:

| Trigger | Reaction |
|---|---|
| Emergency Stop | Cat 1 |
| Safeguard Stop (external protective device) | Cat 2 |
| 3PE Stop | Cat 2 |
| **Limit violation** (any safety limit) | **Cat 0** |
| **Fault detection** (incl. cut wire in E-stop circuit) | **Cat 0** |

Cat 1 monitored-stop numbers: brake initiation ≤ **24 ms**, full stop within **500 ms** at constant deceleration, power-off finalize ≤ **600 ms**, at-rest drift ≤ **0.05 rad**; failure → escalate to Cat 0.

### 3.2 Protective stop vs safety violation — two different animals [V]

- **Controller "Protective Stop" (C150–C164)**: the controller detects it can't follow the planned trajectory (collision, wrong payload/CoG, cable drag, cold start, singularity, pushing). Robot **decelerates along the path, power stays on**, program pauses. Recovery requires deliberate user action (never auto-reset): close popup → `unlock protective stop` → `play`, resuming **from where it stopped**. C163: >50 protective stops on the same joint in 8 h → 5 s unlock throttle.
- **Safety violation/fault**: **Cat 0** → power off, brakes engage, **Recovery mode** with fixed limits (joint 30 °/s, TCP 250 mm/s, force 100 N, momentum 10 kg·m/s, power 80 W); recovery = `restart safety` → power on → brake release → play, program **restarts from the beginning**.

### 3.3 Safety function tolerances & defaults [V / L]

Monitoring tolerances (UR5e manual §10.2.3): joint position 5°, joint speed 1.15 °/s, safety planes 40 mm, tool orientation 3°, TCP speed 50 mm/s, force 25 N, momentum 3 kg·m/s, power 10 W, stopping time 50 ms, stopping distance 40 mm.

e-series factory presets (extracted from PolyScope `safety.conf`; labeled "suggestions only"): TCP/elbow speed 1500 (Normal) / 750 (Reduced) mm/s, force 150/120 N, power 300/200 W, momentum 25/10 kg·m/s, stopping time 400/300 ms, stopping distance 500/300 mm.

**Collision detection is sensorless** (motor current + dual encoders — UR has no torque sensors). The "65 N / 100 N" figures circulating online are NOT UR defaults (65 N is the ISO/TS 15066 contact limit; 100 N is the CB3 minimum force limit). e-Series default tool-force limit is 150 N; the joint-level monitor is a **torque window** ("exceeded torque window by {float} Nm", C283A26) — no published default window value in Nm [NF].

### 3.4 Plant-relevant error code families [V]

- **Protective stops:** C153A0–A5 position deviates from path (per joint); C155 wrong payload; C156 wrong mounting/Freedrive push; C157/C158 collision detected by joint; C162 wrong payload mass/CoG; C161 large movement while powered off; C163 throttle.
- **Safety violations (Cat 0):** C191A1 joint position, A2 joint speed, A3 TCP speed, A4 TCP position, **A6 power**, **A7/A8 joint torque window**, **A12 momentum**, A14 not braking in stop mode, A15 moving while stopped, A16 did not stop in time.
- **Electrical:** C50A5 PSU low; C50A51/A52 robot voltage below/above threshold; C53A IO overcurrent (800/600 mA); **C56A overvoltage (>55 V, regen)**; **C59A overcurrent shutdown** ("if more than two times in a row, replace joint"); C296A44 24 V out of range.
- **Encoders:** C74/C75 (warning/error) sub-flags: invalid decode, speed invalid, supply voltage, temperature, signal lost/low/saturation, "position not changing while motor runs"; C77/C78 sudden position change; C46A "loose gearbox or bad encoder mounting"; e-Series C263/C265 equivalents.
- **Brakes:** C57A brake release failure; C73A brake test failed during selftest; C62A1 hot joint (brake drag is a documented cause).
- **Comms:** C49A RS485, C76A joint-encoder CRC ("heavy electrical noise"), C103A SCB link lost, e-Series C262/C276/C299 groups.
- **Selftest:** C190A (motor-encoder index, phases, no movement); C187A temperature sensor test.

### 3.5 Physical failure modes and symptoms

| Failure | Mechanism | Documented symptom | Source |
|---|---|---|---|
| Harmonic drive wear | flexspline crack/pitting, lubricant degradation | C46A, C70A "close to gearbox shear limit", C77/C78 sudden position changes, **recurrent C153 stops** under acceleration; literature adds torque ripple, vibration, motor-vs-joint encoder divergence, current spikes | [V] service manual + [L] [Guida et al., Machines 2024](https://www.mdpi.com/2075-1702/12/11/776) |
| Encoder fault | read-head gap/misalignment, stray field, metal particles | C75A/C265A sub-flags; intermittent "position changed unexpectedly"; batch defect 2017–2019 (C75) → joint replacement | [V] [C265 page](https://www.universal-robots.com/manuals/EN/HTML/SW10_15/Content/prod-err-codes/topics/CODE_265.html) |
| Brake drag / release failure | solenoid fault, 24 V sag, worn pads | C57A (won't move / large movement during release), C62A1 hot joint; failed brake on vertical axis → **joint drops** (3rd-party) | [V] service manual + 3rd-party |
| Cable / connector | flex-cycle damage, EMC noise | intermittent comms faults under motion (C49A/C76A), C50A100 | [V] |
| Power supply | PSU aging, IO short, regen overvoltage | C50A5, C56A, C59A, C296A44 | [V] |

**Not found [NF]:** vendor MTBF / gearbox service intervals; per-joint brake inventory in print; exact default torque-window Nm; vendor use of the word "drift" (UR says "position deviates from path").

---

## 4. Implications for the plant model (proposed plan adjustments)

### 4.1 Grounded parameter defaults (UR5e)

| Plant parameter | Value to use | Provenance |
|---|---|---|
| Effort saturation | 150/150/150/28/28/28 Nm | [V] |
| Velocity limits | 3.1416 rad/s (180 °/s) all joints | [V] |
| Gear ratio (motor-side modeling) | 101:1 | [L] |
| Motor lag (first-order τ) | ~5 ms per joint (≈1–3 ms electrical + 2 ms loop) | [L]+[V] estimate |
| Joint friction | Coulomb 7–12 Nm + viscous 9–15 N·m·s/rad (ranges; per-joint split TBD — try to obtain the Gnad 2026 table) | [L] |
| Joint encoder quantization | ~1e-5 rad | derived estimate |
| Joint encoder noise | σ ≈ 0.5–1e-5 rad | estimate [NF] |
| Joint states / FTS publish rate | 500 Hz | [V] |
| FTS range / precision / accuracy | ±50 N / ±10 Nm; ±3.5 N / ±0.2 Nm; ±4.0 N / ±0.3 Nm | [V] |

### 4.2 Plan changes (mapping to plan doc components)

1. **Component 2 (controllers):** keep 500 Hz CM; the JSB must publish at the full 500 Hz (real driver behavior). Effort limits enter as JTC/motor saturation, not just URDF limits.
2. **Component 3 (URDF/description):** friction values now grounded (Coulomb/viscous per-joint from §4.1); acceleration limits remain [NF] — use conservative limits derived from speed/stop-time specs and mark as estimate.
3. **Component 4 (plant_sensors):** encoder quantization 1e-5 rad (was 2π/2^17 ≈ 4.8e-5); publish at 500 Hz. FTS: add **saturation at ±50 N/±10 Nm**, **relative-sensor zeroing service** (`/plant/ft_sensor/zero`, mirroring `~/zero_ftsensor`), slow bias drift; noise σ anchored to precision envelope (σ_F ≈ 1 N, σ_T ≈ 0.05 Nm conservative, or the literature 0.3 N + 100 Hz LP). FTS rate 500 Hz (was 100 Hz in plan).
4. **Component 5 (actuator plugin):** add gear-ratio-aware motor-side modeling (motor inertia + 101:1) if we want the two-mass effect; at minimum keep τ≈5 ms lag + rate limit + saturation at vendor torques. Servo bandwidth remains a tuning target, not a documented constant.
5. **Component 6 (fault injection):** replace generic fault list with **code-grounded taxonomy** (below); optionally emulate the protective-stop vs Cat-0 distinction and the C163 throttle.
6. **Plan doc correction:** "no IMU on real UR" is wrong — e-series has internal base IMUs + tool accelerometer (specs unpublished). Keep them out of the exposed suite; note tool accelerometer as optional future work.

### 4.3 Fault taxonomy v2 (each grounded in a documented mechanism)

| Fault (service name) | Real mechanism / code | Injectable effect | Expected signature |
|---|---|---|---|
| `gearbox_wear` | harmonic drive degradation (C46A, C70A, recurrent C153) | friction ramp + small backlash + torque ripple | growing tracking error, effort ripple at gear frequency |
| `joint_jam` | obstruction/stall (C157, C59A) | velocity forced to 0 / braking torque | effort spike, velocity collapse |
| `brake_drag` | partially engaged brake (C57A, C62A1) | extra constant drag + heat input | one-sided tracking error, hot joint |
| `torque_derate` | thermal derating / weak motor | effort scale factor < 1 | larger tracking error under load |
| `payload_misconfig` | wrong payload/CoG (C155/C156/C162) | attach offset-mass payload | gravity-induced error at some poses only |
| `collision_push` | external contact (C157/C158, C191A6/A7/A12) | impulsive/sustained external force on TCP | effort spike, path deviation, stop |
| `encoder_fault` | read-head/magnetic fault (C75A, C265A) | dropout, bias jump, spike, saturation | stale/jumping joint states |
| `fts_zero_shift` | FTS zeroing lost (C450) | step bias in wrench | wrong wrench until re-zero |
| `voltage_sag` | 24 V rail brownout (C296A44, C50A51) | reduced torque capacity + comms glitches | degraded tracking, intermittent dropouts |
| `joint_overheat` | duty-cycle thermal model (C62A1/A11) | temp rise → warning 80 °C → shutdown 85 °C, no auto-recovery | temp topic, then stop |
| `comms_latency` | UART/RS485 degradation (C49A, C76A) | added latency/jitter on sensor topics | stale data, twin lag |

### 4.4 Deliberately NOT modeled (for now)

- Full safety state machine (stop categories, Recovery mode, safety.conf limits) — large scope; only add if twin scenarios need "robot stops and requires recovery". The fault framework will expose a `severity` field so it can be added later.
- Two-mass motor/flexspline dynamics — reflected motor inertia through 101:1 can be folded into τ≈5 ms lag; full model only if gearbox-wear signatures need it.
- Thermal physics (conduction model) — a simple first-order duty-cycle→temperature model per joint is enough to hit the documented thresholds.
- Base IMUs / tool accelerometer — no vendor specs exist to ground them.

### 4.5 Still open

- Exact per-joint friction table (Gnad et al. 2026 — paywalled; ranges only for now).
- Servo position bandwidth: undocumented → tune τ/rate-limits empirically so step responses look plausible (finite rise time, slight overshoot, ~50–200 ms settling at 500 Hz loop).
- FTS noise spectrum: pick conservative vendor-anchored values vs literature values.
- Whether to emulate protective-stop/Cat-0 semantics + C163 throttle (Phase 3 candidate).

## Sources (primary)

- UR5e User Manual v5.8 PDF — <https://s3-eu-west-1.amazonaws.com/ur-support-site/69091/99404_UR5e_User_Manual_en_Global.pdf>
- e-Series Service Manual v1.1.6 (mirror) — <http://www.dour-robots.com/upload/down/UR%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%9C%8D%E5%8A%A1%E6%89%8B%E5%86%8C/ServiceManual_EN_e-Series_V_1.1.6.pdf>
- Current e-series datasheets — <https://www.universal-robots.com/manuals/latest/en/datasheets/ur3e/> / `ur5e` / `ur10e` / `ur16e`
- UR "Max. joint torques" — <https://www.universal-robots.com/articles/ur-articles/max-joint-torques/>
- UR error-code handbooks (SW5_26 / SW10_15) — <https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-err-codes/err-ErrorCodes.htm> and per-code pages (`CODE_62`, `CODE_265`, `CODE_157`, `CODE_296`, `CODE_353`, `CODE_363`, ...)
- RTDE guide — <https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/>
- "Understanding Protective Stops" — <https://www.universal-robots.com/articles/ur/robot-care-maintenance/understanding-protective-stops/>
- UR max-temperature article — <https://www.universal-robots.com/articles/ur/application-installation/what-is-the-maximum-temperature-the-robot-can-work-in/>
- Universal_Robots_ROS2_Driver (configs/docs) — <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>
- Renishaw "AksIM supports Universal Robots" — <https://www.renishaw.com/nl/aksim-supports-universal-robots-for-smart-factory-automation--40389>
- Clochiatti et al., "Electromechanical modeling and identification of the UR5 e-Series robot", Robotica 2024 — <https://www.cambridge.org/core/journals/robotica/article/1AE5BAE866D904F6C79B4C159BEA2B45F>
- Gnad et al., IEEE Access 2026 (UR5e friction) — <https://doi.org/10.1109/ACCESS.2026.3726682>
- Kovincic et al. 2019 (UR5 CB3 identification, friction temperature dependence) — <https://www.semanticscholar.org/paper/3efd8f18fc54b1e0f628c0db0c2f158848881c41>
- Madsen PhD thesis (Aarhus, e-series joint architecture, stiffness) — <https://pure.au.dk/ws/files/197603831/Phd_Thesis_Emil_Madsen.pdf>
- Guida et al., "Comprehensive Analysis of Major Fault-to-Failure Mechanisms in Harmonic Drives", Machines 2024 — <https://www.mdpi.com/2075-1702/12/11/776>
- PoliTo theses (UR5 joint encoders, gearbox faults) — <https://webthesis.biblio.polito.it/16939/1/tesi.pdf>, <https://webthesis.biblio.polito.it/22398/1/tesi.pdf>
- UPC thesis (PolyScope safety.conf presets) — <https://upcommons.upc.edu/bitstream/handle/2117/353369/tfg-lluis-bonet-ortu-o.pdf>
