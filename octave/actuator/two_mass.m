% two_mass.m — two-mass mechanical ODE: rotor + stiffening flexspline + link.
% States x = [theta_m; w_m; theta_j; w_j] (rad, rad/s).
% Input: tau_em, motor-side torque (Nm). The flexspline transmits the joint
% side with windup phi = theta_m/N - theta_j:
%   tau_fs = k0*phi + k2*phi^3 + d*phi_dot      (stiffening spring + damping)
% Joint-side friction uses the Gnad per-joint values with smoothed Coulomb;
% the link carries the gravity load tau_g (test input).
function dx = two_mass(x, tau_em, M)
  th_m = x(1); w_m = x(2); th_j = x(3); w_j = x(4);
  phi      = th_m/M.N - th_j;
  phi_dot  = w_m/M.N - w_j;
  tau_fs   = M.k0*phi + M.k2*phi^3 + M.d*phi_dot;      % joint-side, Nm
  tau_fric = M.pC*tanh(w_j/M.eps) + M.pv*w_j;          % Gnad, joint side
  dw_m = (tau_em - tau_fs/M.N - M.pC_m*tanh(w_m/M.eps) - M.pv_m*w_m)/M.Jm;
  dw_j = (tau_fs - tau_fric - M.tau_g)/M.Jl;
  dx = [w_m; dw_m; w_j; dw_j];
end
