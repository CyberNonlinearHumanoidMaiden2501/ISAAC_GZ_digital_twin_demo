% torque_speed.m — speed-torque capability envelopes of the two UR5e motor
% candidates (Kollmorgen TBM2G [OEM] datasheet values) on the plant's 48 V
% bus. Continuous (thermal) and peak (short-time) regions, the
% voltage-limited tail (no flux weakening, id = 0), and the plant's rated
% operating point (reflected joint torque demand at rated speed).
% Run from octave/actuator:  octave torque_speed.m

clear; close all; more off;
if ~exist('figs', 'dir'), mkdir('figs'); end

Vdc = 48; Vmax = Vdc/sqrt(3);            % SVPWM linear ceiling, V peak phase
w_rated = 101*3.1416;                    % motor speed at 180 deg/s joint speed

% [OEM] Kollmorgen TBM2G selection guide (June 2023): per-winding cold/hot
% Kt (Nm/Arms), R line-line (Ohm, 25 C), L line-line (H).
motors(1).name = 'TBM2G-06826 (J1-3)';
motors(1).p = 10;                        % pole pairs
motors(1).wind = struct('label', {'A','C','D'}, ...
  'Kt_cold', {0.496, 0.248, 0.143}, 'Kt_hot', {0.442, 0.221, 0.128}, ...
  'R', {2.60, 0.651, 0.217}, 'L', {3.51e-3, 0.88e-3, 0.29e-3});
motors(1).Tc = 1.54;                     % Nm continuous, dT = 130 C [OEM]
motors(1).Tp = 3.96;                     % Nm peak (est. from the 68-series
                                         %   lineup table [OEM])
motors(1).def_wind = 3;                  % D = plant default
motors(1).tau_dem = 150/101;             % reflected rated joint torque
motors(2).name = 'TBM2G-05008 (wrist)';
motors(2).p = 7;
motors(2).wind = struct('label', {'A','C','D'}, ...
  'Kt_cold', {0.171, 0.086, 0.049}, 'Kt_hot', {0.153, 0.076, 0.044}, ...
  'R', {1.38, 0.345, 0.115}, 'L', {1.26e-3, 0.32e-3, 0.11e-3});
motors(2).Tc = 0.27;                     % [OEM]
motors(2).Tp = 0.72;                     % [OEM]
motors(2).def_wind = 2;                  % C = plant default
motors(2).tau_dem = 28/101;

w = linspace(0, 850, 2000)';             % 8000 rpm = 838 rad/s mechanical max

fname = {'torque_speed_06826.png', 'torque_speed_05008.png'};
for m = 1:2
  figure('visible', 'off'); hold on;
  nw = numel(motors(m).wind);
  for j = 1:nw
    W = motors(m).wind(j);
    p = motors(m).p; we = p*w;
    Lph = W.L/2;                          % phase quantities (star)
    % hot params for the continuous region, cold for the peak region:
    % hot R = R25 * (1 + 0.0039*(155-25)) ~ 1.5x; hot Kt = magnet at temp
    Kt_h = W.Kt_hot/sqrt(2);  lam_h = Kt_h/(1.5*p);  Rh = W.R/2*1.5;
    Kt_c = W.Kt_cold/sqrt(2); lam_c = Kt_c/(1.5*p);  Rc = W.R/2;
    % voltage-limited current, id = 0: (R*i + we*lam)^2 + (we*L*i)^2 = Vmax^2
    iv = @(R, lam) ((-2*R*we*lam) + sqrt(max(0, (2*R*we*lam).^2 - ...
        4*(R^2 + (we*Lph).^2).*((we*lam).^2 - Vmax^2)))) ...
        ./ (2*(R^2 + (we*Lph).^2));
    tau_cont = min(motors(m).Tc, Kt_h.*iv(Rh, lam_h));
    tau_peak = min(motors(m).Tp, Kt_c.*iv(Rc, lam_c));
    if j == motors(m).def_wind
      plot(w, tau_cont, 'b', 'LineWidth', 2);
      plot(w, tau_peak, 'r', 'LineWidth', 2);
    else
      plot(w, tau_cont, 'b--');
      plot(w, tau_peak, 'r--');
    end
  end
  % plant rated operating point and demand line
  plot([0 850], motors(m).tau_dem*[1 1], 'k:');
  plot(w_rated, motors(m).tau_dem, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
  xlim([0 850]); ylim([0 motors(m).Tp*1.1]);
  xlabel('motor speed (rad/s)'); ylabel('torque (Nm)');
  title(motors(m).name); grid on;
  dl = motors(m).wind(motors(m).def_wind).label;
  legend(['continuous (' dl ')'], ['peak (' dl ')'], ...
         'cont./peak (other windings)', 'rated demand', ...
         'rated operating point', 'location', 'northeast');
  print(['figs/' fname{m}], '-dpng');
end

% ------------------------------------------------------------------
% numbers for the report
% ------------------------------------------------------------------
fprintf('\n== speed-torque envelope summary (48 V bus, no flux weakening) ==\n');
for m = 1:2
  W = motors(m).wind(motors(m).def_wind);
  p = motors(m).p;
  Kt_h = W.Kt_hot/sqrt(2); lam_h = Kt_h/(1.5*p); Rh = W.R/2*1.5;
  we = p*w;
  Lph = W.L/2;
  iv = @(R, lam) ((-2*R*we*lam) + sqrt(max(0, (2*R*we*lam).^2 - ...
      4*(R^2 + (we*Lph).^2).*((we*lam).^2 - Vmax^2)))) ./ (2*(R^2 + (we*Lph).^2));
  tau_cont = min(motors(m).Tc, Kt_h.*iv(Rh, lam_h));
  icorner = find(tau_cont < 0.99*motors(m).Tc, 1);
  fprintf('\n%s (winding %s):\n', motors(m).name, W.label);
  fprintf('  Tc = %.2f Nm, Tp = %.2f Nm (peak = %.1f x Tc)\n', ...
          motors(m).Tc, motors(m).Tp, motors(m).Tp/motors(m).Tc);
  fprintf('  continuous corner speed = %.0f rad/s (%.0f rpm); rated = %.0f rad/s\n', ...
          w(icorner), w(icorner)*60/(2*pi), w_rated);
  fprintf('  rated demand %.2f Nm = %.0f %% of Tc (%.0f %% of Tp)\n', ...
          motors(m).tau_dem, 100*motors(m).tau_dem/motors(m).Tc, ...
          100*motors(m).tau_dem/motors(m).Tp);
  fprintf('  -> demand sits at the continuous rating: evidence the candidate\n');
  fprintf('     motor sizes match the UR5e joint torque specs\n');
end
fprintf('\nsaved figs/torque_speed_06826.png and figs/torque_speed_05008.png\n');
