% test_electrical.m — validates the electrical stage of the L2 actuator
% reference model (d-q PMSM + FOC current loop).
% Run from octave/actuator:  octave test_electrical.m
% Rotor speed is an external input throughout (M.Jm = Inf): the mechanical
% stage (two_mass.m) replaces this stand-in in 1.2.

clear; close all; more off;
if ~exist('figs', 'dir'), mkdir('figs'); end

M1 = params('j1');       % shoulder/elbow-class motor
Mw = params('wrist');

fprintf('\n== parameter sanity ==\n');
fprintf('J1-3 : L/R = %.2f ms (research 1.3 ms), Kt = %.3f Nm/A, lambda = %.3f Wb\n', ...
        M1.L/M1.R*1e3, M1.Kt, M1.lambda);
fprintf('wrist: L/R = %.2f ms (research 0.9 ms), Kt = %.3f Nm/A\n', ...
        Mw.L/Mw.R*1e3, Mw.Kt);
fprintf('vmax = %.1f V (Vdc/sqrt(3), Vdc = %.0f V), fsw = %.0f Hz\n', ...
        M1.vmax, M1.Vdc, M1.fsw);

M = M1; dt = M.dt;
M.Jm = Inf;               % electrical stage: wm is an external input

% ------------------------------------------------------------------
% 1) open-loop q-voltage step, locked rotor -> tau_e = L/R, i_ss = V/R
% ------------------------------------------------------------------
n1 = round(10e-3/dt);                     % 10 ms
x = [0; 0; 0]; iq_ol = zeros(n1,1);
for k = 1:n1
  x = rk4_step(x, 0, 5, 0, M, dt);        % 5 V step on vq, wm = 0
  iq_ol(k) = x(2);
end
t1 = (1:n1)'*dt*1e3;                      % ms
[~,i63] = min(abs(iq_ol - (1-exp(-1))*iq_ol(end)));
fprintf('\n[1] open-loop step (locked rotor):\n');
fprintf('    i_ss = %.4f A (V/R = %.3f), measured tau_e = %.2f ms (target 1.3)\n', ...
        iq_ol(end), 5/M.R, t1(i63));
fprintf('    tau_em = Kt*i_q = %.3f Nm (%.3f expected)\n', M.Kt*iq_ol(end), M.Kt*5/M.R);

% ------------------------------------------------------------------
% 2) closed current loop: iq* step 0 -> 5 A, locked rotor
% ------------------------------------------------------------------
x = [0; 0; 0]; s = zeros(4,1);
n2 = round(20e-3/dt);
t2 = (1:n2)'*dt*1e3;
iq_cl = zeros(n2,1); vq_cl = zeros(n2,1);
for k = 1:n2
  iq_star = 5*(k*dt > 2e-3);
  [vd, vq, s] = foc_loop(x(1), x(2), x(3), 0, iq_star, M, s);
  x = rk4_step(x, vd, vq, 0, M, dt);
  iq_cl(k) = x(2); vq_cl(k) = vq;
end
i10 = find(iq_cl >= 0.5, 1);
i90 = find(iq_cl >= 4.5, 1);
peak = max(iq_cl);
iset = find(t2 > 2, 1);                   % after the step
err = abs(iq_cl(iset:end) - 5);
its = find(err > 0.1, 1, 'last');         % 2 % band
tset = ifelse(isempty(its), 0, its*dt*1e3);
fprintf('\n[2] current-loop step 0->5 A (target crossover %.0f Hz):\n', M.wc/(2*pi));
fprintf('    rise 10-90%% = %.2f ms, overshoot = %.1f %%, peak = %.2f A\n', ...
        (i90-i10)*dt*1e3, (peak-5)/5*100, peak);
fprintf('    settle (2%%)  = %.2f ms\n', tset);

% ------------------------------------------------------------------
% 3) back-EMF at speed, open loop, v = 0
% ------------------------------------------------------------------
wm0 = 300;
we = M.p*wm0;
n3 = round(20e-3/dt);
x = [0; 0; wm0];
for k = 1:n3
  x = rk4_step(x, 0, 0, 0, M, dt);        % held speed, no voltage
end
% coupled 2x2 steady state (id-iq cross-coupling through we*L)
iq_ss = -we*M.lambda*M.R/(M.R^2 + (we*M.L)^2);
id_ss =  we*M.L*iq_ss/M.R;
fprintf('\n[3] back-EMF at wm = %.0f rad/s, v = 0:\n', wm0);
fprintf('    iq -> %.3f A (coupled ss = %.3f), id -> %.3f A (ss = %.3f)\n', ...
        x(2), iq_ss, x(1), id_ss);

% ------------------------------------------------------------------
% 4) flux-weakening / voltage-ceiling check
% ------------------------------------------------------------------
wm_rated = M.N*3.1416;                    % motor speed at 180 deg/s joint speed
wm_nofw  = M.vmax*1.5/M.Kt;               % speed where back-EMF reaches the ceiling (iq=0)
if wm_nofw > wm_rated
  fw = 'no flux weakening needed';
else
  fw = 'FLUX WEAKENING REQUIRED';
end
fprintf('\n[4] flux-weakening check:\n');
fprintf('    rated motor speed = %.0f rad/s (joint 180 deg/s through %d:1)\n', ...
        wm_rated, M.N);
fprintf('    no-FW max speed   = %.0f rad/s (back-EMF = vmax at iq = 0)\n', wm_nofw);
fprintf('    margin = %.2f x -> %s\n', wm_nofw/wm_rated, fw);

% simulated ceiling: closed loop holding iq* = 0 while speed ramps up
n4 = round(0.5/dt);
wm_ramp = linspace(0, 600, n4)';
x = [0; 0; 0]; s = zeros(4,1); vq_hist = zeros(n4,1);
for k = 1:n4
  [vd, vq, s] = foc_loop(x(1), x(2), x(3), 0, 0, M, s);
  x = rk4_step(x, vd, vq, 0, M, dt);
  x(3) = wm_ramp(k);
  vq_hist(k) = vq;
end
kclamp = find(abs(vq_hist) >= M.vmax*0.999, 1);
fprintf('    simulated ceiling: vq hits vmax at wm = %.0f rad/s (analytic %.0f)\n', ...
        wm_ramp(kclamp), wm_nofw);

% ------------------------------------------------------------------
% figure for the user
% ------------------------------------------------------------------
figure('visible', 'off');
subplot(3,1,1);
plot(t1, iq_ol); ylabel('i_q (A)'); title('open-loop q step, locked rotor');
grid on; xlabel('t (ms)');
subplot(3,1,2);
plot(t2, iq_cl); hold on;
plot(t2, 5*(t2 > 2), 'k--');
ylabel('i_q (A)'); title('current-loop step 0->5 A'); legend('i_q', 'i_q*');
grid on; xlabel('t (ms)');
subplot(3,1,3);
plot(wm_ramp, vq_hist, 'b', [0 600], M.vmax*[1 1], 'r--');
xlabel('w_m (rad/s)'); ylabel('v_q (V)'); title('iq* = 0 speed ramp -> voltage ceiling');
grid on;
print('figs/test_electrical.png', '-dpng');
fprintf('\nsaved figs/test_electrical.png\n');
