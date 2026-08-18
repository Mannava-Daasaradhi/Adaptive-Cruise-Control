function [val, wpk] = hinf(kat, kv, kp, hw, tau, omega)
%HINF  ||Htilde(j*omega; tau)||_inf on a frequency grid  (Definition 2, eq. 3).
%
%   [val, wpk] = MA2025.HINF(kat, kv, kp, hw, tau)
%   [val, wpk] = MA2025.HINF(kat, kv, kp, hw, tau, omega)
%
%   Robust string stability holds iff val <= 1.  WPK is the peak frequency,
%   useful for sanity checks: for this family the peak sits at omega = 0 or
%   in 0.02..1 rad/s, well inside the default grid.
%
%   Default grid: logspace(-3, 2.5, 8000) rad/s, matched to the Python
%   reference implementation (src/cacc/analysis.py, OMEGA_DEFAULT) so the
%   two backends report identical numbers.  omega = 0 is appended because
%   |Htilde(0)| = kp/kp = 1 exactly and the sup is often attained there.
%
%   See also MA2025.HTILDE, MA2025.MIN_STABLE_HEADWAY.

arguments
    kat   (1,1) double
    kv    (1,1) double
    kp    (1,1) double
    hw    (1,1) double
    tau   (1,1) double
    omega (1,:) double = [0, logspace(-3, 2.5, 8000)]
end

[~, ~, H] = ma2025.Htilde(kat, kv, kp, hw, tau, omega);
[val, ix] = max(abs(H));
wpk = omega(ix);
end
