% sense_current.m — the drive's current MEASUREMENT model: what the twin
% sees on the virtual motor-current stream (a real UR `tool_data` channel).
%
% Physics note: real drives sample the current at the PWM midpoint, which
% rejects the f_sw ripple — the measurement is the period AVERAGE. So the
% default measurement = average + 12-bit ADC quantization + bias, with NO
% ripple. The ripple's analytic bound (Delta_i_pp = Vdc*Tsw*m/(8*L), m =
% modulation index) is available as an optional statistics-correct synthetic
% injection (S.ripple_on) for experiments that want a noisy current signal;
% it never affects the dynamics (signal-only path).
%
% State S: Vdc (mirror of the inverter bus), I_range (ADC full scale),
% bias [E], ripple_on (flag).
function [idm, iqm] = sense_current(id, iq, vd, vq, S, M)
  m = min(1, sqrt(vd^2 + vq^2)/(S.Vdc/sqrt(3)));   % modulation index
  if S.ripple_on
    % analytic ripple std (triangle p-p -> std = pp/(2*sqrt(3))), white noise
    pp = S.Vdc*M.Tsw*m/(8*M.L);
    rip = pp/(2*sqrt(3))*randn();
  else
    rip = 0;
  end
  lsb = 2*S.I_range/4096;                % 12-bit ADC
  idm = round((id + rip)/lsb)*lsb + S.bias;
  iqm = round((iq + rip)/lsb)*lsb + S.bias;
end
