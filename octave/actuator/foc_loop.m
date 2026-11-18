% foc_loop.m — discrete d-q current controller with FOC decoupling.
% Runs once per switching period (10 kHz). The commanded voltage is held over
% the period (ZOH, implicit in the fixed-step integration) and applied one
% sample late (computation delay), giving ~1.5*Tsw total loop lag.
%
% Controller state s = [int_d; int_q; vd_prev; vq_prev].
% Returns [vd, vq, s] for the voltage to apply during the NEXT period.
function [vd, vq, s] = foc_loop(id, iq, wm, id_star, iq_star, M, s)
  we = M.p*wm;
  e_d = id_star - id;
  e_q = iq_star - iq;
  s(1) += M.ki*e_d*M.dt;   % q/d integrators
  s(2) += M.ki*e_q*M.dt;
  vd_raw = M.kp*e_d + s(1) - we*M.L*iq;               % decoupling
  vq_raw = M.kp*e_q + s(2) + we*(M.L*id + M.lambda);  % decoupling + back-EMF FF
  % circular voltage clamp (SVPWM ceiling) with conditional-integration
  % anti-windup
  vmag = sqrt(vd_raw^2 + vq_raw^2);
  if vmag > M.vmax
    vd_raw *= M.vmax/vmag;
    vq_raw *= M.vmax/vmag;
    s(1) -= M.ki*e_d*M.dt;
    s(2) -= M.ki*e_q*M.dt;
  end
  % one-sample computation delay: emit the previous command
  vd = s(3);
  vq = s(4);
  s(3) = vd_raw;
  s(4) = vq_raw;
end
