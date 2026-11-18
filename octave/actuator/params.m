% params.m — per-joint actuator parameter seeds for the L2 reference model.
% Provenance: [L] literature, [V] vendor, [E] engineering estimate seed.
% The [E] seeds are placeholders until the Kollmorgen TBM2G datasheet values
% are sourced (Stage 2 research sub-task). Their ratios are pinned to the
% research values: tau_e = L/R = 1.3 ms (J1-3) / 0.9 ms (wrist).
%
% Usage: M = params('j1')   % shoulder/elbow-class motor
%        M = params('wrist')
function M = params(joint_group)
  M.Vdc   = 48.0;            % DC bus voltage, V [V: 48 V-class, regen clamp 55 V]
  M.fsw   = 10000;           % PWM switching frequency, Hz [E]
  M.Tsw   = 1/M.fsw;         % switching period, s
  M.vmax  = M.Vdc/sqrt(3);   % SVPWM linear ceiling, V peak phase
  M.p     = 4;               % pole pairs [E seed]
  M.N     = 101;             % gear ratio [L]
  M.dt    = M.Tsw;           % fixed integration step = one switching period
  if strcmp(joint_group, 'j1')
    M.R  = 0.40;             % phase resistance, Ohm [E]
    M.L  = 0.52e-3;          % phase inductance, H [E] -> tau_e = 1.3 ms [L]
    M.Kt = 0.12;             % torque constant, Nm/A [E]
    M.Jm = 6.4e-5;           % rotor inertia, kg m^2 [L] Robotica 2024
  else
    M.R  = 0.55;             % Ohm [E]
    M.L  = 0.50e-3;          % H [E] -> tau_e = 0.9 ms [L]
    M.Kt = 0.08;             % Nm/A [E]
    M.Jm = 1.2e-5;           % kg m^2 [L]
  end
  M.lambda = M.Kt/(1.5*M.p); % flux linkage, Wb (surface PMSM: Kt = 1.5 p lambda)
  % current-loop PI gains: magnitude-optimum for the RL plant at crossover wc.
  % wc = f_sw/20 = 500 Hz keeps phase margin >= 60 deg against the 1.5*Tsw
  % delay + ZOH (800 Hz gives ~47 deg PM / ~29 % overshoot — the hot option).
  M.wc     = 2*pi*500;       % target current-loop crossover, rad/s [E]
  M.kp     = M.L*M.wc;       % V/A
  M.ki     = M.R*M.wc;       % V/(A s)
end
