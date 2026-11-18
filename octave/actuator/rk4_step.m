% rk4_step.m — one fixed-step RK4 integration step of the PMSM ODE.
% Mirrors the fixed-step scheme the future gz-sim plugin will use.
function x1 = rk4_step(x0, vd, vq, tau_load, M, dt)
  k1 = pmsm(x0, vd, vq, tau_load, M);
  k2 = pmsm(x0 + dt/2*k1, vd, vq, tau_load, M);
  k3 = pmsm(x0 + dt/2*k2, vd, vq, tau_load, M);
  k4 = pmsm(x0 + dt*k3, vd, vq, tau_load, M);
  x1 = x0 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
