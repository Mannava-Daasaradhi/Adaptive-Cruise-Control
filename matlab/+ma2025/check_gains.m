function R = check_gains(ka, rho, kv, kp, hw, tau0)
%CHECK_GAINS  Full Theorem 2 admissibility audit of one design (ka,kv,kp,hw).
%
%   R = MA2025.CHECK_GAINS(ka, rho, kv, kp, hw, tau0)
%
%   Evaluates, and reports individually, every condition the paper imposes:
%
%     (16)  0 < ka < 1/(1 + 1/rho)                 feedforward gain cap
%     (17)  hw > h_lb(ka, rho)                     robust headway bound
%     (28a) gamma <= (1 - (1+1/rho)^2 ka^2)/(2 tau0)   high-noise end
%     (28b) gamma >= sqrt(2 kp (1 - (1-1/rho) ka) + kv^2)  low-noise end
%     (RH)  gamma > tau0 * kp                      internal stability
%                                                  (Routh-Hurwitz on D(s))
%
%   plus the DIRECT numerical verdict ||Htilde||_inf <= 1 taken as the worst
%   case over the noise interval I -- which is the ground truth.  Conditions
%   (28a)/(28b) are SUFFICIENT, not necessary: they come from demanding each
%   power of omega^2 in eq. (25) be non-negative separately, which discards
%   the possibility that a positive omega^6 term rescues a slightly negative
%   omega^4 one.  Designs can therefore fail (28) and still be string stable;
%   R.sufficient_ok and R.hinf_ok are reported separately for exactly this
%   reason.
%
%   See also MA2025.FEASIBLE_SET, MA2025.HINF_WORST.

arguments
    ka   (1,1) double
    rho  (1,1) double
    kv   (1,1) double
    kp   (1,1) double
    hw   (1,1) double
    tau0 (1,1) double = 0.5
end

m   = 1 - 1/rho;
n   = (1 + 1/rho)^2;
gam = kv + hw * kp;

R.ka = ka; R.rho = rho; R.kv = kv; R.kp = kp; R.hw = hw; R.tau0 = tau0;
R.gamma = gam;

% (16) feedforward cap ---------------------------------------------------
R.ka_max  = 1 / (1 + 1/rho);
R.eq16_ok = (ka > 0) && (ka < R.ka_max);

% (17) headway lower bound ----------------------------------------------
if R.eq16_ok
    R.h_lb = ma2025.h_lb(ka, rho, tau0);
else
    R.h_lb = NaN;
end
R.eq17_ok = hw > R.h_lb;

% (28a) high-noise-end ceiling on gamma (independent of hw!) -------------
R.gamma_max = (1 - n * ka^2) / (2 * tau0);
R.eq28a_ok  = gam <= R.gamma_max;

% (28b) low-noise-end floor on gamma ------------------------------------
R.gamma_min = sqrt(2 * kp * (1 - m * ka) + kv^2);
R.eq28b_ok  = gam >= R.gamma_min;

% Routh-Hurwitz internal stability of D(s) = tau s^3 + s^2 + gamma s + kp
% a1 a2 > a0 a3  <=>  1*gamma > kp*tau, worst at tau = tau0.
R.rh_margin = gam - tau0 * kp;
R.rh_ok     = R.rh_margin > 0;

R.sufficient_ok = R.eq16_ok && R.eq17_ok && R.eq28a_ok && R.eq28b_ok && R.rh_ok;

% Ground truth ----------------------------------------------------------
[R.hinf_worst, R.kat_binding, R.omega_peak] = ...
    ma2025.hinf_worst(ka, rho, kv, kp, hw, tau0);
R.hinf_low  = ma2025.hinf(ma2025.ka_tilde(ka, rho, 'low'),  kv, kp, hw, tau0);
R.hinf_mean = ma2025.hinf(ma2025.ka_tilde(ka, rho, 'mean'), kv, kp, hw, tau0);
R.hinf_high = ma2025.hinf(ma2025.ka_tilde(ka, rho, 'high'), kv, kp, hw, tau0);
R.hinf_ok   = R.hinf_worst <= 1 + 1e-9;
end
