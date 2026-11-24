% rk4_mech.m — one fixed-step RK4 integration step of the two-mass ODE.
function x1 = rk4_mech(x0, tau_em, M, dt)
  k1 = two_mass(x0, tau_em, M);
  k2 = two_mass(x0 + dt/2*k1, tau_em, M);
  k3 = two_mass(x0 + dt/2*k2, tau_em, M);
  k4 = two_mass(x0 + dt*k3, tau_em, M);
  x1 = x0 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
