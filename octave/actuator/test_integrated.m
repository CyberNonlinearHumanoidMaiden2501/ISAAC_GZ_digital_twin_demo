% test_integrated.m — full-chain open-loop verification (1.3b):
% foc_loop -> inverter -> pmsm -> two_mass + Gnad friction, with the bus
% state. No position loop (that is Step 2 against Gazebo) — this proves the
% reference model is consistent end to end.
% Run from octave/actuator:  octave test_integrated.m

clear; close all; more off;
if ~exist('figs', 'dir'), mkdir('figs'); end

M = params_mech('j1', 1);               % pan joint: pC 8.58, pv 13.9
dt = M.dt;
I = struct('Vdc', 48, 'Vsrc', 48, 'Rsrc', 0.05, 'C', 10e-3, ...
           't_dt', 1e-6, 'eps_s', 0.5, 'V_clamp', 55);

n = round(2/dt);
x = zeros(6,1); s = zeros(4,1);
wj = zeros(n,1); phi = zeros(n,1); iq = zeros(n,1); tau_fs_h = zeros(n,1);
for k = 1:n
  iq_star = 3*(k*dt > 0.2);
  [vd_cmd, vq_cmd, s] = foc_loop(x(1), x(2), x(4), 0, iq_star, M, s);
  [vd, vq, I] = inverter(vd_cmd, vq_cmd, x(1), x(2), M.p*x(3), I, M, dt);
  x = rk4_plant(x, vd, vq, M, dt);
  wj(k) = x(6); iq(k) = x(2);
  ph = x(3)/M.N - x(5);
  phi(k) = ph;
  tau_fs_h(k) = M.k0*ph + M.k2*ph^3 + M.d*(x(4)/M.N - x(6));
end
w_ss = (M.N*M.Kt*3 - M.pC)/M.pv;
fprintf('\n[5] integrated chain, iq* = 3 A step (joint 1):\n');
fprintf('    i_q tracks %.3f A (commanded 3)\n', iq(end));
fprintf('    terminal w_j = %.4f rad/s (expected (N*Kt*iq - pC)/pv = %.4f)\n', ...
        wj(end), w_ss);
fprintf('    steady tau_fs = %.2f Nm (= pC + pv*w_j = %.2f)\n', ...
        tau_fs_h(end), M.pC + M.pv*wj(end));
fprintf('    max windup during accel = %.3f mrad, steady phi_dot = %.2e rad/s\n', ...
        max(phi)*1e3, x(4)/M.N - x(6));
fprintf('    bus Vdc = %.2f V after the run (source droop under load)\n', I.Vdc);

figure('visible', 'off');
t = (1:n)'*dt;
subplot(3,1,1);
plot(t, iq); ylabel('i_q (A)'); title('current loop through the full chain');
grid on; xlabel('t (s)');
subplot(3,1,2);
plot(t, wj); ylabel('w_j (rad/s)');
title('link speed: friction-limited terminal velocity'); grid on; xlabel('t (s)');
subplot(3,1,3);
plot(t, phi*1e3); ylabel('phi (mrad)');
title('flexspline windup during acceleration'); grid on; xlabel('t (s)');
print('figs/test_integrated.png', '-dpng');
fprintf('\nsaved figs/test_integrated.png\n');
