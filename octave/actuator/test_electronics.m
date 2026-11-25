% test_electronics.m — validates the SVPWM/drive-electronics layer (1.3a):
% the measurement model (midpoint-sampled average + ADC + bias), the
% dead-time 6th-harmonic disturbance, the bus regen clamp, and the
% voltage-sag torque falloff.
% Run from octave/actuator:  octave test_electronics.m

clear; close all; more off;
if ~exist('figs', 'dir'), mkdir('figs'); end

M = params('j1'); M.Jm = Inf; dt = M.dt;
I0 = struct('Vdc', 48, 'Vsrc', 48, 'Rsrc', 0.05, 'C', 10e-3, ...
            't_dt', 2e-6, 'eps_s', 0.5, 'V_clamp', 55);   % 0.5-2 us typical;
                                                           % 2 us for a visible
                                                           % dead-time signature
S0 = struct('Vdc', 48, 'I_range', 30, 'bias', 0.02, 'ripple_on', false);

% ------------------------------------------------------------------
% 1) measurement model: quantization + bias (+ optional ripple stats)
% ------------------------------------------------------------------
wm = 300;                                % high modulation -> large ripple
n1 = round(0.5/dt);
[iqt, iqm, vqm, ~] = run_el(wm, 5, I0, S0, M, dt, n1, 0.1);
i1 = round(0.3/dt);                      % steady window
lsb = 2*S0.I_range/4096;
err = iqm(i1:end) - iqt(i1:end);
m_mod = min(1, abs(vqm(end))/(I0.Vdc/sqrt(3)));
pp = I0.Vdc*M.Tsw*m_mod/(8*M.L);
S_on = S0; S_on.ripple_on = true;
[~, iqm_rip, ~, ~] = run_el(wm, 5, I0, S_on, M, dt, n1, 0.1);
err_rip = iqm_rip(i1:end) - iqt(i1:end);
fprintf('\n[1] measurement model (wm = %.0f rad/s, iq* = 5 A):\n', wm);
fprintf('    bias = %.4f A (set %.4f), std = %.4f A (ADC lsb/sqrt(12) = %.4f)\n', ...
        mean(err), S0.bias, std(err), lsb/sqrt(12));
fprintf('    modulation index m = %.2f -> analytic ripple pp = %.2f A\n', m_mod, pp);
fprintf('    ripple_on: measured std = %.3f A (analytic pp/2/sqrt(3) = %.3f)\n', ...
        std(err_rip), pp/(2*sqrt(3)));
fprintf('    dynamics unaffected: i_q still tracks %.3f A (measurement is signal-only)\n', ...
        iqt(end));

% ------------------------------------------------------------------
% 2) dead-time 6th harmonic in v_q (low current -> dead-time dominates)
% ------------------------------------------------------------------
n2 = round(1/dt);                        % 1 s -> 0.1 Hz bins
[~, ~, vq_on, ~]  = run_el(100, 1, I0, S0, M, dt, n2, 0.2);
Ioff = I0; Ioff.t_dt = 0;
[~, ~, vq_off, ~] = run_el(100, 1, Ioff, S0, M, dt, n2, 0.2);
Fs = 1/dt; Nf = 2^nextpow2(n2);
f = (0:Nf/2-1)*Fs/Nf;
Vq_on  = abs(fft(vq_on  - mean(vq_on),  Nf)); Vq_on  = Vq_on(1:Nf/2);
Vq_off = abs(fft(vq_off - mean(vq_off), Nf)); Vq_off = Vq_off(1:Nf/2);
f_e = M.p*100/(2*pi); f6 = 6*f_e;
i6 = find(f >= f6*0.9 & f <= f6*1.1);
fprintf('\n[2] dead-time 6th harmonic (f_e = %.0f Hz -> 6*f_e = %.0f Hz):\n', f_e, f6);
fprintf('    v_q amplitude near 6*f_e: t_dt = 2 us -> %.3f V, t_dt = 0 -> %.3f V\n', ...
        max(Vq_on(i6))*2/n2, max(Vq_off(i6))*2/n2);

% ------------------------------------------------------------------
% 3) bus regen: generating current pumps the bus up to the 55 V clamp
% ------------------------------------------------------------------
n3 = round(0.3/dt);
I = I0; Vdc_hist = zeros(n3,1);
x = [0;0;200]; s = zeros(4,1);
for k = 1:n3
  [vd_cmd, vq_cmd, s] = foc_loop(x(1), x(2), x(3), 0, -2, M, s);
  th_e = M.p*200*k*dt;
  [vd, vq, I] = inverter(vd_cmd, vq_cmd, x(1), x(2), th_e, I, M, dt);
  x = rk4_step(x, vd, vq, 0, M, dt);
  x(3) = 200;
  Vdc_hist(k) = I.Vdc;
end
fprintf('\n[3] regen at wm = 200 rad/s, iq* = -2 A:\n');
fprintf('    Vdc %.1f -> %.2f V (clamp 55 V = C56A regen overvoltage)\n', ...
        Vdc_hist(1), Vdc_hist(end));

% ------------------------------------------------------------------
% 4) voltage sag: bus droops -> ceiling drops -> torque falloff at speed
% ------------------------------------------------------------------
Isag = I0; Isag.Vsrc = 40;              % sag fault hook (C296A44/C50A51)
n4 = round(0.5/dt);
[iqt4, ~, ~, I4] = run_el(330, 5, Isag, S0, M, dt, n4, 0.1);
[iqt4h, ~, ~, ~] = run_el(330, 5, I0, S0, M, dt, n4, 0.1);
fprintf('\n[4] voltage sag (Vsrc = 40 V) at wm = 330 rad/s, iq* = 5 A:\n');
fprintf('    bus sags to %.2f V -> ceiling %.2f V (healthy %.2f)\n', ...
        I4.Vdc, I4.Vdc/sqrt(3), I0.Vdc/sqrt(3));
fprintf('    i_q achieves %.2f A (healthy bus: %.2f A) -> torque falloff %.0f %%\n', ...
        iqt4(end), iqt4h(end), 100*(1 - iqt4(end)/iqt4h(end)));

% ------------------------------------------------------------------
% figure for the user
% ------------------------------------------------------------------
figure('visible', 'off');
subplot(2,2,1);
plot((1:n1)'*dt*1e3, iqt, 'b', (1:n1)'*dt*1e3, iqm, 'r.');
xlabel('t (ms)'); ylabel('i_q (A)'); title('true vs measured current (ADC)');
legend('true', 'measured'); grid on;
subplot(2,2,2);
plot(f, 2*Vq_on/n2, 'b', f, 2*Vq_off/n2, 'r');
xlim([0 2500]); xlabel('f (Hz)'); ylabel('v_q amplitude (V)');
title('dead-time 6th harmonic in v_q'); legend('t_dt = 2 us', 't_dt = 0');
grid on;
subplot(2,2,3);
plot((1:n3)'*dt*1e3, Vdc_hist); xlabel('t (ms)'); ylabel('V_dc (V)');
title('regen -> bus clamp at 55 V'); grid on;
subplot(2,2,4);
plot((1:n4)'*dt*1e3, iqt4, 'r', (1:n4)'*dt*1e3, iqt4h, 'b');
xlabel('t (ms)'); ylabel('i_q (A)');
title('sag: torque falloff at speed'); legend('sagged bus', 'healthy');
grid on;
print('figs/test_electronics.png', '-dpng');
fprintf('\nsaved figs/test_electronics.png\n');
