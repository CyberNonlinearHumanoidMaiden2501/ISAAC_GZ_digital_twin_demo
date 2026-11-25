% plant_ode.m — the FULL integrated plant ODE: d-q electrical (i_d, i_q) +
% two-mass mechanical (theta_m, w_m, theta_j, w_j).
% x = [id; iq; th_m; w_m; th_j; w_j]; inputs vd, vq (post-inverter).
function dx = plant_ode(x, vd, vq, M)
  id = x(1); iq = x(2); wm = x(4);
  we = M.p*wm;
  did = (vd - M.R*id + we*M.L*iq)/M.L;
  diq = (vq - M.R*iq - we*M.L*id - we*M.lambda)/M.L;
  tau_em = 1.5*M.p*M.lambda*iq;
  dxm = two_mass(x(3:6), tau_em, M);
  dx = [did; diq; dxm];
end
