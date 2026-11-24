% params.m — per-joint actuator parameters for the L2 reference model.
%
% TWO VIEWS OF THE MACHINE:
%   PLANT truth (physics):     R, L, lambda, Kt, p, Jm ...
%   CONTROLLER belief (FOC):   Rhat, Lhat, lambdahat, Kthat
% Real drives identify the motor at commissioning (selftest/auto-tune) and
% those measurements carry errors — the FOC therefore never has the exact
% plant. Belief = truth * (1 + id_err), with id_err bounded by the datasheet
% tolerances. pmsm.m uses only truth; foc_loop.m uses only belief.
%
% PROVENANCE (verification notes):
%   [OEM] Kollmorgen TBM2G datasheets (selection guide + installation manual,
%         June 2023), cold (25 C) values; https://www.kollmorgen.com
%         TBM2G-06826 (68 mm OD, 26.3 mm stack, 20 poles) — J1-3 candidate
%         TBM2G-05008 (50 mm OD,  8.2 mm stack, 14 poles) — wrist candidate
%   [L]   Robotica 2024, "Electromechanical modeling and identification of
%         the UR5 e-Series robot": tau_e = L/R ~ 1.3 ms (J1-3) / ~0.9 ms
%         (wrist); rotor inertia Jm = 6.4e-5 / 1.2e-5 kg m^2.
%         CROSS-CHECK: L_ll/R_ll = 1.34-1.35 ms for ALL 06826 windings and
%         0.91-0.96 ms for ALL 05008 windings -> research tau_e verified.
%         (Note: two datasheet revisions disagree slightly on the 06826
%         winding tables; both give the same tau_e, so the choice below is
%         insensitive to the revision.)
%   [V]   bus: C56A regen overvoltage trips above 55 V -> 48 V-class DC bus
%         (inferred from the error-code threshold; UR publishes no bus spec).
%   [L]   gear ratio N = 101 (Gnad et al. 2026, from UR config files).
%   [E]   fsw = 10 kHz (UR publishes nothing; typical industrial servo),
%         wc = f_sw/20 design point (PM >= 60 deg against 1.5*Tsw delay).
%
% WINDING CHOICE (documented default; revisit when the Robotica paper's
% winding selection is known): at rated motor speed (N * 180 deg/s =
% 317 rad/s), the 06826 A/C windings' back-EMF exceeds the Vdc/sqrt(3) =
% 27.7 V ceiling — without field weakening the J1-3 motor must be winding D
% (A: 74 V, C: 37 V, D: 21 V back-EMF at rated speed). The 05008 A winding
% fits the bus only at 0.93 utilization (no headroom for load at speed); C
% is the max-Kt choice with >= 20 % headroom.
%   Defaults: J1-3 -> 06826-D, wrist -> 05008-C.
%
% Model conventions: the d-q equations use PHASE quantities (star
% connection): R = R_ll/2, L = L_ll/2 -> tau_e = L/R = L_ll/R_ll (ratio
% invariant). Currents are PEAK values: Kt = Kt_rms/sqrt(2),
% lambda = Kt/(1.5*p), and tau_em = 1.5*p*lambda*iq = Kt*iq.

function M = params(joint_group)
  % --- plant truth (physics) ---
  M.Vdc   = 48.0;            % [V] see provenance above
  M.fsw   = 10000;           % [E] PWM switching frequency, Hz
  M.Tsw   = 1/M.fsw;         % switching period, s
  M.vmax  = M.Vdc/sqrt(3);   % SVPWM linear ceiling, V peak phase
  M.N     = 101;             % [L] gear ratio
  M.dt    = M.Tsw;           % fixed integration step
  if strcmp(joint_group, 'j1')
    % TBM2G-06826 winding D [OEM]: R_ll = 0.217 (+-10 %), L_ll = 0.29 mH
    % (+-20 %), Kt_rms = 0.143 Nm/Arms, Ke = 8.65 Vrms/krpm, 20 poles.
    M.p      = 10;           % pole pairs [OEM]
    M.R      = 0.217/2;      % phase resistance, Ohm [OEM]
    M.L      = 0.29e-3/2;    % phase inductance, H [OEM] -> tau_e = 1.34 ms
    M.Kt_rms = 0.143;        % Nm/Arms [OEM] cold
    M.Jm     = 6.4e-5;       % rotor inertia, kg m^2 [L] (datasheet rotor
                             %   5.18e-5 kg m^2 + hub/brake disk)
  else
    % TBM2G-05008 winding C [OEM]: R_ll = 0.345 (+-10 %), L_ll = 0.32 mH
    % (+-20 %), Kt_rms = 0.086 Nm/Arms, Ke = 5.18 Vrms/krpm, 14 poles.
    M.p      = 7;            % pole pairs [OEM]
    M.R      = 0.345/2;      % phase resistance, Ohm [OEM]
    M.L      = 0.32e-3/2;    % phase inductance, H [OEM] -> tau_e = 0.93 ms
    M.Kt_rms = 0.086;        % Nm/Arms [OEM] cold
    M.Jm     = 1.2e-5;       % rotor inertia, kg m^2 [L] (datasheet rotor
                             %   7.9e-6 kg m^2 + hub/brake disk)
  end
  M.Kt     = M.Kt_rms/sqrt(2);   % Nm/A_peak (peak-current convention)
  M.lambda = M.Kt/(1.5*M.p);     % flux linkage, Wb
  % --- controller belief (identification results, with errors) ---
  % THREE error regimes exist; id_err models the FIRST as the healthy default:
  %  1. identification accuracy (what modern drive ID achieves): R ~1-3 %,
  %     L ~5-15 %, Kt/lambda ~2-5 % (DC step + HF injection + back-EMF at
  %     speed). DEFAULT below.
  %  2. unit tolerance (datasheet spread, worst case if the drive never
  %     identifies the unit): Kt/R +-10 %, L +-20 % [OEM].
  %     Worst-case corners for experiments: R +10 %, L -20 %, Kt +10 %.
  %  3. thermal drift: R +0.39 %/C, Kt -11 % cold->hot [OEM hot/cold
  %     columns] — modeled later on the TRUTH side as R(T)/lambda(T) hooks
  %     in the thermal model, not as a static id_err.
  M.id_err = struct('R', 0.03, 'L', 0.08, 'Kt', 0.03);
  M.Rhat      = M.R*(1+M.id_err.R);
  M.Lhat      = M.L*(1+M.id_err.L);
  M.Kthat     = M.Kt*(1+M.id_err.Kt);
  M.lambdahat = M.Kthat/(1.5*M.p);
  % --- current-loop design, computed from the BELIEF (as a real drive would) ---
  M.wc     = 2*pi*500;       % [E] target crossover = f_sw/20, rad/s
  M.kp     = M.Lhat*M.wc;    % V/A
  M.ki     = M.Rhat*M.wc;    % V/(A s)
end
