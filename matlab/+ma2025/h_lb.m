function h = h_lb(ka, rho, tau0)
%H_LB  Robust time-headway lower bound, Theorem 2(b), eq. (17).
%
%   h = MA2025.H_LB(ka, rho, tau0)
%
%                              1 - (1 - 1/rho) ka
%       h_{w,lb}(ka) = 2 tau0 --------------------                        (17)
%                             1 - (1 + 1/rho)^2 ka^2
%
%   Valid only for 0 < ka < 1/(1 + 1/rho)  (eq. 16), which keeps the
%   denominator positive.  Reduces to the noiseless bound 2 tau0/(1 + ka)
%   of Darbha-Konduri-Pagilla as rho -> Inf (Remark 4).
%
%   Derivation sketch (see docs/derivations Part II, S2.7):
%     Theorem 1(b) needs h_w > 2 tau0 / (1 + k_a_tilde) for the chosen
%     k_a_tilde; robustness demands the worst (largest) such bound over
%     k_a_tilde in I, i.e. the SMALLEST admissible k_a_tilde = (1-1/rho) ka
%     in the numerator, while the feasibility of (kv,kp) simultaneously
%     forces the (1+1/rho)^2 ka^2 term in the denominator via (28a).
%
%   See also MA2025.KA_STAR, MA2025.H_STAR, MA2025.CHECK_GAINS.

arguments
    ka   (1,:) double
    rho  (1,1) double {mustBeGreaterThan(rho,1)}
    tau0 (1,1) double {mustBePositive} = 0.5
end

kaMax = 1 / (1 + 1/rho);
if any(ka <= 0) || any(ka >= kaMax)
    error('ma2025:h_lb:range', ...
        'eq. (16) requires 0 < ka < 1/(1+1/rho) = %.6f; got min %.6f max %.6f', ...
        kaMax, min(ka), max(ka));
end

num = 1 - (1 - 1/rho) .* ka;
den = 1 - (1 + 1/rho)^2 .* ka.^2;
h   = 2 * tau0 * num ./ den;
end
