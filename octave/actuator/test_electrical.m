% test_electrical.m — validates the electrical stage of the L2 actuator
% reference model (d-q PMSM + FOC current loop).
% Run from octave/actuator:  octave test_electrical.m
% Rotor speed is an external input throughout (M.Jm = Inf): the mechanical
% stage (two_mass.m) replaces this stand-in in 1.2.
% Scenarios 2/5 contrast the FOC's BELIEF (identification results with
% errors) against the plant's TRUTH — the FOC never has perfect parameters.

clear; close all; more off;
if ~exist('figs', 'dir'), mkdir('figs'); end

M1 = params('j1');       % shoulder/elbow-class motor
Mw = params('wrist');

fprintf('\n== parameter sanity (verified against [OEM] datasheets) ==\n');
fprintf('J1-3 : tau_e = L/R = %.2f ms (research 1.3, datasheet L_ll/R_ll 1.34-1.35)\n', ...
        M1.L/M1.R*1e3);
fprintf('       R=%.4f L=%.4f mH Kt=%.4f Nm/Ap p=%d lambda=%.5f Wb\n', ...
        M1.R, M1.L*1e3, M1.Kt, M1.p, M1.lambda);
fprintf('wrist: tau_e = L/R = %.2f ms (research 0.9, datasheet L_ll/R_ll 0.91-0.96)\n', ...
        Mw.L/Mw.R*1e3);
fprintf('       R=%.4f L=%.4f mH Kt=%.4f Nm/Ap p=%d lambda=%.5f Wb\n', ...
        Mw.R, Mw.L*1e3, Mw.Kt, Mw.p, Mw.lambda);
fprintf('vmax = %.1f V (Vdc/sqrt(3), Vdc = %.0f V), fsw = %.0f Hz\n', ...
        M1.vmax, M1.Vdc, M1.fsw);
fprintf('FOC belief (ID errors R +%.0f%%, L %+.0f%%, Kt +%.0f%%):\n', ...
        M1.id_err.R*100, M1.id_err.L*100, M1.id_err.Kt*100);
fprintf('  Rhat=%.4f Lhat=%.4f mH lambdahat=%.5f Wb\n', ...
        M1.Rhat, M1.Lhat*1e3, M1.lambdahat);

M = M1; dt = M.dt;
M.Jm = Inf;               % electrical stage: wm is an external input

% ------------------------------------------------------------------
% 1) open-loop q-voltage step, locked rotor -> tau_e = L/R, i_ss = V/R
% ------------------------------------------------------------------
n1 = round(10e-3/dt);                     % 10 ms
x = [0; 0; 0]; iq_ol = zeros(n1,1);
for k = 1:n1
  x = rk4_step(x, 0, 1, 0, M, dt);        % 1 V step on vq, wm = 0
  iq_ol(k) = x(2);
end
t1 = (1:n1)'*dt*1e3;                      % ms
[~,i63] = min(abs(iq_ol - (1-exp(-1))*iq_ol(end)));
fprintf('\n[1] open-loop step (locked rotor):\n');
fprintf('    i_ss = %.4f A (V/R = %.4f), measured tau_e = %.2f ms (target 1.34)\n', ...
        iq_ol(end), 1/M.R, t1(i63));
fprintf('    tau_em = Kt*i_q = %.4f Nm (%.4f expected)\n', M.Kt*iq_ol(end), M.Kt*1/M.R);

% ------------------------------------------------------------------
% 2) closed current loop, DEFAULT identification errors: iq* 0 -> 5 A
% ------------------------------------------------------------------
x = [0; 0; 0]; s = zeros(4,1);
n2 = round(20e-3/dt);
t2 = (1:n2)'*dt*1e3;
iq_cl = zeros(n2,1);
for k = 1:n2
  iq_star = 5*(k*dt > 2e-3);
  [vd, vq, s] = foc_loop(x(1), x(2), x(3), 0, iq_star, M, s);
  x = rk4_step(x, vd, vq, 0, M, dt);
  iq_cl(k) = x(2);
end
i10 = find(iq_cl >= 0.5, 1);
i90 = find(iq_cl >= 4.5, 1);
peak = max(iq_cl);
iset = find(t2 > 2, 1);
err = abs(iq_cl(iset:end) - 5);
its = find(err > 0.1, 1, 'last');         % 2 % band
tset = ifelse(isempty(its), 0, its*dt*1e3);
fprintf('\n[2] current-loop step 0->5 A, ID errors (design crossover %.0f Hz):\n', ...
        M.wc/(2*pi));
fprintf('    achieved crossover ~ %.0f Hz (Lhat/L mismatch: wc*Lhat/L)\n', ...
        M.wc*M.Lhat/M.L/(2*pi));
fprintf('    rise 10-90%% = %.2f ms, overshoot = %.1f %%, peak = %.2f A, settle (2%%) = %.2f ms\n', ...
        (i90-i10)*dt*1e3, (peak-5)/5*100, peak, tset);
fprintf('    steady-state error = %.1e A (integral action -> 0 despite ID errors)\n', ...
        iq_cl(end) - 5);

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
fprintf('\n[3] back-EMF at wm = %.0f rad/s (we = %.0f), v = 0:\n', wm0, we);
fprintf('    iq -> %.3f A (coupled ss = %.3f), id -> %.3f A (ss = %.3f)\n', ...
        x(2), iq_ss, x(1), id_ss);

% ------------------------------------------------------------------
% 4) voltage ceiling: winding table + simulated ceiling
% ------------------------------------------------------------------
wm_rated = M.N*3.1416;                    % motor speed at 180 deg/s joint speed
fprintf('\n[4] winding vs voltage ceiling at rated speed %.0f rad/s (vmax %.1f V):\n', ...
        wm_rated, M.vmax);
% Kt_rms per winding [OEM]: 06826 A/C/D and 05008 A/C/D
Kt_wind = [0.496 0.248 0.143;             % J1-3 (TBM2G-06826)
           0.171 0.086 0.049];            % wrist (TBM2G-05008)
wname = {'j1-06826', 'wrist-05008'};
wlbl = {'A', 'C', 'D'};                   % datasheet winding names
for m = 1:2
  for w = 1:3
    Ktr = Kt_wind(m,w);
    bemf = Ktr*wm_rated/(1.5*sqrt(2));    % back-EMF phase peak at rated speed
    wm_nofw = M.vmax*1.5*sqrt(2)/Ktr;
    if bemf < M.vmax
      fits = sprintf('fits (%.2f x bus)', M.vmax/bemf);
    else
      fits = 'EXCEEDS ceiling -> needs field weakening';
    end
    fprintf('    %-10s winding %s: Kt=%.3f, back-EMF %.1f V -> %s\n', ...
            wname{m}, wlbl{w}, Ktr, bemf, fits);
  end
end
% simulated ceiling with the default plant (06826-D): iq* = 0 speed ramp
wm_nofw = M.vmax*1.5/M.Kt;                % analytic no-FW speed for default Kt
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
fprintf('    default plant (06826-D): ceiling at wm = %.0f rad/s (analytic %.0f), margin %.2f x\n', ...
        wm_ramp(kclamp), wm_nofw, wm_nofw/wm_rated);

% ------------------------------------------------------------------
% 5) perfect identification vs errors; lambda-error headroom cost
% ------------------------------------------------------------------
Mp = M;                                   % perfect-ID copy
Mp.Rhat = Mp.R; Mp.Lhat = Mp.L; Mp.Kthat = Mp.Kt; Mp.lambdahat = Mp.lambda;
Mp.kp = Mp.Lhat*Mp.wc; Mp.ki = Mp.Rhat*Mp.wc;
x = [0; 0; 0]; s = zeros(4,1); iq_perf = zeros(n2,1);
for k = 1:n2
  iq_star = 5*(k*dt > 2e-3);
  [vd, vq, s] = foc_loop(x(1), x(2), x(3), 0, iq_star, Mp, s);
  x = rk4_step(x, vd, vq, 0, Mp, dt);
  iq_perf(k) = x(2);
end
peak_p = max(iq_perf);
fprintf('\n[5] perfect ID vs errors:\n');
fprintf('    perfect ID: overshoot = %.1f %% (errors case: %.1f %%)\n', ...
        (peak_p-5)/5*100, (peak-5)/5*100);
fprintf('    torque-path gain error (Kt/Kthat - 1) = %+.1f %% — the Kt-ID error\n', ...
        (M.Kt/M.Kthat - 1)*100);
fprintf('    shows up HERE (every effort command is scaled by Kt/Kthat),\n');
fprintf('    not in the current loop (its SSE is 0 via integral action)\n');
dv = wm0*M.p*(M.lambdahat - M.lambda);    % feedforward error at speed
fprintf('    lambda-ID error %+.0f%% consumes %.1f V of the %.1f V budget at wm = %.0f rad/s\n', ...
        M.id_err.Kt*100, dv, M.vmax, wm0);

% ------------------------------------------------------------------
% figure for the user
% ------------------------------------------------------------------
figure('visible', 'off');
subplot(3,1,1);
plot(t1, iq_ol); ylabel('i_q (A)'); title('open-loop q step, locked rotor');
grid on; xlabel('t (ms)');
subplot(3,1,2);
plot(t2, iq_cl, 'b', t2, iq_perf, 'r--', t2, 5*(t2 > 2), 'k:');
ylabel('i_q (A)'); title('current-loop step 0->5 A');
legend('i_q (ID errors)', 'i_q (perfect ID)', 'i_q*');
grid on; xlabel('t (ms)');
subplot(3,1,3);
plot(wm_ramp, vq_hist, 'b', [0 600], M.vmax*[1 1], 'r--');
xlabel('w_m (rad/s)'); ylabel('v_q (V)'); title('iq* = 0 speed ramp -> voltage ceiling');
grid on;
print('figs/test_electrical.png', '-dpng');
fprintf('\nsaved figs/test_electrical.png\n');
