% inverter.m — power stage between the current controller and the motor:
% circular clamp at the ACTUAL bus, dead-time voltage error in d-q, and the
% bus-capacitor state (source droop + regen clamp) that makes voltage_sag
% and regen-overvoltage faults physical.
%
% State I: Vdc (bus), Vsrc (supply, sag fault lowers it), Rsrc, C (bus
% capacitor), t_dt (dead time), eps_s (zero-crossing smoothing),
% V_clamp (regen clamp, 55 V = C56A).
% Inputs: vd_cmd/vq_cmd (controller output), id/iq, th_e = p*theta_m.
% Provenance: [E] C/Rsrc/t_dt seeds (UR publishes nothing); clamp voltage
% [V] C56A regen overvoltage > 55 V.
function [vd, vq, I] = inverter(vd_cmd, vq_cmd, id, iq, th_e, I, M, dt)
  % 1. circular clamp of the COMMAND at the actual bus ceiling
  vmax = I.Vdc/sqrt(3);
  vd = vd_cmd; vq = vq_cmd;
  vmag = sqrt(vd^2 + vq^2);
  if vmag > vmax
    vd *= vmax/vmag;
    vq *= vmax/vmag;
  end
  % 2. dead-time error: per-phase -Vdt*sign(i_phase) (switches lose
  %    t_dt per period, opposing the current), mapped to d-q via the
  %    electrical angle — the 6*f_e harmonic disturbance of real drives
  Vdt = I.Vdc*(I.t_dt/M.Tsw);
  ia = id*cos(th_e) - iq*sin(th_e);
  ib = id*cos(th_e - 2*pi/3) - iq*sin(th_e - 2*pi/3);
  ic = id*cos(th_e + 2*pi/3) - iq*sin(th_e + 2*pi/3);
  ea = -Vdt*tanh(ia/I.eps_s);     % tanh = smoothed zero-crossing clamp
  eb = -Vdt*tanh(ib/I.eps_s);
  ec = -Vdt*tanh(ic/I.eps_s);
  vd += 2/3*(ea*cos(th_e) + eb*cos(th_e - 2*pi/3) + ec*cos(th_e + 2*pi/3));
  vq -= 2/3*(ea*sin(th_e) + eb*sin(th_e - 2*pi/3) + ec*sin(th_e + 2*pi/3));
  % 3. bus capacitor: C*dV/dt = (P_src - P_out)/V. The supply is
  %    diode-isolated: it sources power only while Vdc < Vsrc, so regen
  %    has nowhere to go but the clamp (the C56A behavior).
  P_src = max(0, I.Vsrc*(I.Vsrc - I.Vdc)/I.Rsrc);  % sag = lower Vsrc
  P_out = 1.5*(vd*id + vq*iq);               % power into the motor (d-q)
  I.Vdc += (P_src - P_out)/I.Vdc/I.C*dt;
  I.Vdc = min(max(I.Vdc, 0), I.V_clamp);     % regen clamp at 55 V (C56A)
end
