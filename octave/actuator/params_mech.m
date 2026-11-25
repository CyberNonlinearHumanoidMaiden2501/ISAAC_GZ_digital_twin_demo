% params_mech.m — mechanical-stage parameters: two-mass flexspline + joint
% friction, per joint. Extends the electrical params (Jm, N shared).
%
% PROVENANCE:
%   [L] Gnad et al. 2026 friction table (docs/sources/literature/
%       friction_table.xlsx): UR5e mean of two robots, w.r.t. the gearbox
%       output. pv viscous (N·m·s/rad), pC Coulomb (Nm) per joint:
%         joint:     1      2      3      4      5      6
%         pv:     13.9    8.9    8.92   1.26   1.59   1.39
%         pC:      8.58  12.94   9.56   2.24   2.08   2.13
%   [E] flexspline stiffness k0: seeds (J1-3: 1.5e4, wrist: 8e3 Nm/rad).
%       The Madsen thesis carries the identified UR5e curve only as plots
%       (no numeric readout) — figure extraction abandoned 2026-11-25.
%       Anchor: the UR patent example (CN114867584A, from the thesis) gives
%       k1 ~ 42 Nm/mrad ~ 4.2e4 Nm/rad, k3 ~ 2.4 Nm/mrad^3 ~ 2.4e9 (base
%       joint); Harmonic-Drive catalog units in the UR5e sizes span
%       ~1.4e4-2.9e4. The seeds sit at the low end of that band. Stiffening
%       k2 is set so the torque is (1+s_fac) times the linear value at
%       phi_ref (strain-wave gears are stiffening springs — Madsen).
%   [E] Jl: representative EFFECTIVE link inertia about the joint axis —
%       pose-dependent in reality; Gnad Table 2 has the full rigid-body set
%       for the 1.3 integrated stage.
%   [E] Coulomb smoothing eps: the literature value 1e-3 reproduces a
%       low-speed stick-slip limit cycle in the 1 kHz DART exchange of the
%       Gazebo plugin (2.3 finding); the default 0.05 rad/s keeps the
%       Coulomb character without the hunting. A stiction-aware model
%       (Lugre) is the proper long-term fix. Note: the smoothed form has
%       NO stiction band — static equilibrium sits at tau_fs = tau_g, not
%       tau_g +- pC. True stiction is a later refinement (with asymmetric
%       Coulomb).
%   [E] flexspline damping: modal damping ratio zeta = 0.03 on the two-mass
%       mode (d computed from k0 and the series-combined inertia). No
%       published anchor exists (the thesis models hysteresis, not a
%       viscous ratio) — calibrate during the Step 2.4 tuning session.
function M = params_mech(joint_group, joint_idx)
  M = params(joint_group);               % Jm, N, p, ... (electrical stage)
  pv = [13.9 8.9 8.92 1.26 1.59 1.39];  % [L] Gnad viscous, N·m·s/rad
  pC = [8.58 12.94 9.56 2.24 2.08 2.13];% [L] Gnad Coulomb, Nm
  M.pv = pv(joint_idx);
  M.pC = pC(joint_idx);
  if strcmp(joint_group, 'j1')
    M.k0 = 1.5e4;                        % [E] pending Madsen figure extraction
    M.Jl = 1.0;                          % [E] effective link inertia, kg m^2
  else
    M.k0 = 8e3;                          % [E]
    M.Jl = 0.01;                         % [E]
  end
  M.phi_ref = 5e-3;                      % stiffening reference deflection, rad
  M.s_fac   = 0.5;                       % torque = (1+s_fac)*linear at phi_ref
  M.k2      = M.s_fac*M.k0/M.phi_ref^2;  % Nm/rad^3
  Jm_r      = M.Jm*M.N^2;                % rotor reflected to the joint side
  M.zeta    = 0.03;                      % [E] two-mass mode damping ratio
  M.d       = 2*M.zeta*sqrt(M.k0*Jm_r*M.Jl/(Jm_r + M.Jl));  % N·m·s/rad
  M.pv_m    = 0;                         % motor-side friction (Madsen form
  M.pC_m    = 0;                         %   later) — joint-side [L] is primary
  M.tau_g   = 0;                         % gravity load on the link (test input)
  M.eps     = 0.05;                      % [E] Coulomb smoothing (see header)
end
