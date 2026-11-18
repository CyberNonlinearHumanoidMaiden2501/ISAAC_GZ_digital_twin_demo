% pmsm.m — d-q PMSM electrical ODE (surface PMSM, Ld = Lq = L).
% States x = [id; iq; wm] (A, A, rad/s). Inputs: vd, vq (V), tau_load (Nm,
% motor side). The bare rotor EOM is a stand-in for the electrical-stage
% tests (they lock the rotor); the flexspline coupling replaces it in the
% mechanical stage (two_mass.m).
function dx = pmsm(x, vd, vq, tau_load, M)
  id = x(1); iq = x(2); wm = x(3);
  we = M.p*wm;                       % electrical angular speed, rad/s
  did = (vd - M.R*id + we*M.L*iq)/M.L;
  diq = (vq - M.R*iq - we*M.L*id - we*M.lambda)/M.L;
  dwm = (1.5*M.p*M.lambda*iq - tau_load)/M.Jm;
  dx = [did; diq; dwm];
end
