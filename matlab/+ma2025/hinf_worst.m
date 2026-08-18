function [val, kat_bind, wpk] = hinf_worst(ka, rho, kv, kp, hw, tau, nGrid)
%HINF_WORST  Worst-case ||Htilde||_inf over the whole noise interval I.
%
%   [val, kat_bind, wpk] = MA2025.HINF_WORST(ka, rho, kv, kp, hw, tau)
%
%   Robust string stability (Definition 2 applied to every admissible
%   channel realisation) requires ||Htilde||_inf <= 1 for EVERY
%
%       k_a_tilde in I = [(1 - 1/rho) ka , (1 + 1/rho) ka]      (Sec. III-C)
%
%   so the verdict is the maximum over I, not the value at the mean E[w]*ka.
%   The maximiser is almost always an ENDPOINT of I (|Htilde| is monotone in
%   kat at the peak frequency), but the interior is gridded anyway so the
%   claim is checked rather than assumed.
%
%   KAT_BIND is the binding k_a_tilde.  Project finding (docs D-009): for the
%   paper's designs the LOW end binds, because less feedforward pushes work
%   onto feedback and violates the low-frequency condition (28b) first.

arguments
    ka    (1,1) double
    rho   (1,1) double
    kv    (1,1) double
    kp    (1,1) double
    hw    (1,1) double
    tau   (1,1) double
    nGrid (1,1) double = 41
end

I   = ma2025.ka_tilde(ka, rho, 'interval');
kats = linspace(I(1), I(2), nGrid);
vals = zeros(size(kats));
wpks = zeros(size(kats));
for k = 1:numel(kats)
    [vals(k), wpks(k)] = ma2025.hinf(kats(k), kv, kp, hw, tau);
end
[val, ix] = max(vals);
kat_bind  = kats(ix);
wpk       = wpks(ix);
end
