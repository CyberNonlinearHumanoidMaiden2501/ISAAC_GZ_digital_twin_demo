% run_el.m — closed electrical-chain run with wm held externally:
% foc_loop -> inverter -> pmsm, with the measurement model on the output.
% Used by test_electronics.m.
function [iqt, iqm, vqm, I] = run_el(wm, iq_amp, I, S, M, dt, n, t_step)
  x = [0;0;wm]; s = zeros(4,1);
  iqt = zeros(n,1); iqm = zeros(n,1); vqm = zeros(n,1);
  for k = 1:n
    iq_star = iq_amp*(k*dt > t_step);
    [vd_cmd, vq_cmd, s] = foc_loop(x(1), x(2), x(3), 0, iq_star, M, s);
    th_e = M.p*wm*k*dt;
    [vd, vq, I] = inverter(vd_cmd, vq_cmd, x(1), x(2), th_e, I, M, dt);
    x = rk4_step(x, vd, vq, 0, M, dt);
    x(3) = wm;
    iqt(k) = x(2);
    S.Vdc = I.Vdc;
    [~, iqm(k)] = sense_current(x(1), x(2), vd, vq, S, M);
    vqm(k) = vq;
  end
end
