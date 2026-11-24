% test_mechanical.m — validates the mechanical stage (1.2): the two-mass
% flexspline model with the Gnad per-joint friction.
% Run from octave/actuator:  octave test_mechanical.m
% Scenarios: (1) steady windup under gravity load with stiffening spring,
% (2) anti-resonance/resonance pair (analytic + control-pkg + kick test),
% (3) friction terminal velocity, (4) motor-vs-joint encoder divergence.

clear; close all; more off;
pkg load control;
if ~exist('figs', 'dir'), mkdir('figs'); end

M_pan  = params_mech('j1', 1);   % pv 13.9,  pC  8.58
M_lift = params_mech('j1', 2);   % pv  8.9,  pC 12.94
dt = 1e-4;                       % fixed step (plugin sub-step rate)

% ------------------------------------------------------------------
% 1) steady windup under gravity load (joint 2, like a lift axis)
% ------------------------------------------------------------------
fprintf('\n== parameter sanity (joint 2 lift) ==\n');
fprintf('k0 = %.0f Nm/rad, k2 = %.3g Nm/rad^3 (torque = %.1f x linear at phi_ref = %.1f mrad)\n', ...
        M_lift.k0, M_lift.k2, 1+M_lift.s_fac, M_lift.phi_ref*1e3);
fprintf('d = %.2f N·m·s/rad (zeta = %.2f), Jm*N^2 = %.3f, Jl = %.1f kg m^2\n', ...
        M_lift.d, M_lift.zeta, M_lift.Jm*M_lift.N^2, M_lift.Jl);

n = round(3/dt);
phi_ss = zeros(1,2); phi_lin = zeros(1,2); phi_max = zeros(1,2);
for g = 1:2
  tau_g = 50*g;                          % 50 Nm, 100 Nm
  M = M_lift; M.tau_g = tau_g;
  tau_em = tau_g/M.N;                    % hold it (smoothed Coulomb: no stiction)
  x = [0;0;0;0]; phi = zeros(n,1);
  for k = 1:n
    x = rk4_mech(x, tau_em, M, dt);
    phi(k) = x(1)/M.N - x(3);
  end
  phi_ss(g)  = phi(end);
  phi_lin(g) = tau_g/M.k0;               % what a linear spring would give
  phi_max(g) = max(phi);                 % transient windup (rotor leads)
  fprintf('\n[1] windup, tau_g = %.0f Nm:\n', tau_g);
  fprintf('    phi_ss = %.3f mrad (linear spring would give %.3f mrad, ratio %.2f)\n', ...
          phi_ss(g)*1e3, phi_lin(g)*1e3, phi_ss(g)/phi_lin(g));
  fprintf('    flexspline torque split at phi_ss: linear %.1f Nm + cubic %.1f Nm\n', ...
          M.k0*phi_ss(g), M.k2*phi_ss(g)^3);
  fprintf('    max transient windup (encoder divergence) = %.3f mrad\n', phi_max(g)*1e3);
end
phit = (1:n)'*dt;
% keep first-load trace for the figure and scenario 4
phi_trace = zeros(n,1); M = M_lift; M.tau_g = 50; x = [0;0;0;0];
for k = 1:n
  x = rk4_mech(x, 50/M.N, M, dt);
  phi_trace(k) = x(1)/M.N - x(3);
end

% ------------------------------------------------------------------
% 2) anti-resonance / resonance pair (linear regime, friction zeroed)
% ------------------------------------------------------------------
Mf = M_pan; Mf.pC = 0; Mf.pv = 0; Mf.tau_g = 0; Mf.k2 = 0;   % linearize
Jm_r = Mf.Jm*Mf.N^2; k0 = Mf.k0; d = Mf.d; Jl = Mf.Jl; N = Mf.N; Jm = Mf.Jm;
A = [0 1 0 0;
     -k0/(N^2*Jm)  -d/(N^2*Jm)   k0/(N*Jm)   d/(N*Jm);
      0 0 0 1;
      k0/(N*Jl)     d/(N*Jl)    -k0/Jl      -d/Jl];
B = [0; 1/Jm; 0; 0]; C = [0 0 1 0]; D = 0;
Gm = ss(A,B,[1 0 0 0],0);            % theta_m output (motor encoder side)
Gj = ss(A,B,[0 0 1 0],0);            % theta_j output (joint encoder side)
p = pole(Gm);
[num_m, den] = tfdata(Gm, 'v');      % anti-resonance = zeros of the MOTOR
zr = roots(num_m);                   % response (the link response has none)
w_res_ctrl = abs(imag(p(find(imag(p) > 0, 1))));
w_ar_ctrl  = abs(imag(zr(find(imag(zr) > 0, 1))));
w_ar_form  = sqrt(k0/Jl);
w_res_form = sqrt(k0*(Jm_r+Jl)/(Jm_r*Jl));
% time-domain kick: pure windup IC, zero total momentum
phi0 = 1e-3;
th_j0 = -phi0*Jm_r/(Jm_r+Jl);
th_m0 =  N*phi0*Jl/(Jm_r+Jl);
n2 = round(5/dt);
x = [th_m0; 0; th_j0; 0]; wj = zeros(n2,1);
for k = 1:n2
  x = rk4_mech(x, 0, Mf, dt);
  wj(k) = x(4);
end
Fs = 1/dt; Nf = 2^nextpow2(n2);
W = abs(fft(wj, Nf)); f = (0:Nf/2-1)*Fs/Nf;
[~, ipk] = max(W(1:Nf/2)); fpk = f(ipk);
fprintf('\n[2] resonance pair (linearized, Jl = %.1f):\n', Jl);
fprintf('    anti-resonance: %.2f Hz (formula %.2f, control-pkg %.2f)\n', ...
        w_ar_form/(2*pi), w_ar_form/(2*pi), w_ar_ctrl/(2*pi));
fprintf('    resonance:      %.2f Hz (formula %.2f, control-pkg %.2f, kick-FFT peak %.2f)\n', ...
        w_res_form/(2*pi), w_res_form/(2*pi), w_res_ctrl/(2*pi), fpk);
fprintf('    at the anti-resonance the link+spring act as a vibration absorber:\n');
fprintf('    the ROTOR stands still while the LINK oscillates — the motor\n');
fprintf('    encoder is silent and the joint encoder sees the motion\n');

% ------------------------------------------------------------------
% 3) friction terminal velocity (joint 1, no gravity)
% ------------------------------------------------------------------
M = M_pan; M.tau_g = 0;
tau_em = 0.15;                           % motor torque -> 15.15 Nm at the joint
n3 = round(3/dt);
x = [0;0;0;0]; wj3 = zeros(n3,1);
for k = 1:n3
  x = rk4_mech(x, tau_em, M, dt);
  wj3(k) = x(4);
end
w_ss_expected = (M.N*tau_em - M.pC)/M.pv;
fprintf('\n[3] friction terminal velocity, tau_em = %.2f Nm (joint 1):\n', tau_em);
fprintf('    w_j -> %.4f rad/s (expected (N*tau_em - pC)/pv = %.4f)\n', ...
        wj3(end), w_ss_expected);
fprintf('    steady windup rate phi_dot = %.2e rad/s (constant speed -> 0)\n', ...
        x(2)/M.N - x(4));

% ------------------------------------------------------------------
% 4) motor-vs-joint encoder divergence (from the windup transient)
% ------------------------------------------------------------------
fprintf('\n[4] encoder divergence theta_m/N - theta_j = phi:\n');
fprintf('    peak transient divergence %.3f mrad, steady %.3f mrad at 50 Nm\n', ...
        phi_max(1)*1e3, phi_ss(1)*1e3);
fprintf('    (a real robot reports this as the motor-vs-joint encoder offset;\n');
fprintf('    drift in it is the C46A / gearbox-wear signature)\n');

% ------------------------------------------------------------------
% figure for the user
% ------------------------------------------------------------------
figure('visible', 'off');
subplot(3,1,1);
plot(phit*1e3, phi_trace*1e3); ylabel('phi (mrad)'); xlabel('t (ms)');
title('flexspline windup under a 50 Nm gravity load (joint 2)'); grid on;
subplot(3,1,2);
% manual FRF (avoids bode's singularity at the rigid-body pole at DC):
% motor-side response shows the anti-resonance notch, joint-side does not
ww = logspace(0.2, 3.3, 500);
Hm = polyval(num_m, 1j*ww) ./ polyval(den, 1j*ww);
[num_j, ~] = tfdata(Gj, 'v');
Hj = polyval(num_j, 1j*ww) ./ polyval(den, 1j*ww);
plot(ww/(2*pi), 20*log10(abs(Hm)), 'b', ww/(2*pi), 20*log10(abs(Hj)), 'r--');
xlabel('f (Hz)'); ylabel('magnitude (dB)');
title('two-mass FRF: motor-side notch (anti-resonance) + shared resonance peak');
legend('theta_m (motor encoder)', 'theta_j (joint encoder)'); grid on;
subplot(3,1,3);
plot((1:n3)'*dt, wj3); ylabel('w_j (rad/s)'); xlabel('t (s)');
title('friction-limited terminal velocity (joint 1)'); grid on;
print('figs/test_mechanical.png', '-dpng');
fprintf('\nsaved figs/test_mechanical.png\n');
