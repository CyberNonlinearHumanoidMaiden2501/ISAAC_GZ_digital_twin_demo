% rk4_plant.m — one fixed-step RK4 integration step of the integrated
% plant ODE (6 states).
function x1 = rk4_plant(x0, vd, vq, M, dt)
  k1 = plant_ode(x0, vd, vq, M);
  k2 = plant_ode(x0 + dt/2*k1, vd, vq, M);
  k3 = plant_ode(x0 + dt/2*k2, vd, vq, M);
  k4 = plant_ode(x0 + dt*k3, vd, vq, M);
  x1 = x0 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
