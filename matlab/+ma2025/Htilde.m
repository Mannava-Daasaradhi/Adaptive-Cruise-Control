function [num, den, H] = Htilde(kat, kv, kp, hw, tau, omega)
%HTILDE  Inter-vehicular spacing-error propagation transfer function, eq. (14).
%
%   [num, den]    = MA2025.HTILDE(kat, kv, kp, hw, tau)
%   [num, den, H] = MA2025.HTILDE(kat, kv, kp, hw, tau, omega)
%
%                     Ntilde(s)     k_a_tilde s^2 + kv s + kp
%       Htilde(s;tau) = --------- = ----------------------------          (14)
%                        D(s)       tau s^3 + s^2 + gamma s + kp
%
%   with gamma := kv + hw*kp.  Returned as MATLAB polynomial coefficient
%   vectors (descending powers).  Supplying OMEGA additionally evaluates
%   H = Htilde(j*omega) exactly on the imaginary axis (no Pade, no
%   discretisation), which is what every string-stability verdict uses.
%
%   Derivation (docs/derivations Part II, S2.4).  From eq. (13), Laplace with
%   zero initial conditions and Delta_i = (1 + hw s) X_i - X_{i-1}:
%
%       (tau s^3 + s^2 + kv s) X_i - (kat s^2 + kv s) X_{i-1} = -kp Delta_i
%
%   Eliminating Delta_i gives X_i/X_{i-1} = Ntilde/D directly; since
%   Delta_i = [(1+hw s) Htilde - 1] X_{i-1} and Delta_{i-1} = [(1+hw s) -
%   1/Htilde] X_{i-1}, the ratio Delta_i/Delta_{i-1} equals the SAME
%   Htilde.  Position propagation and spacing-error propagation share one
%   transfer function -- the reason a single scalar bound certifies the string.
%
%   See also MA2025.HINF, MA2025.CHECK_GAINS.

arguments
    kat   (1,1) double
    kv    (1,1) double
    kp    (1,1) double
    hw    (1,1) double
    tau   (1,1) double
    omega (1,:) double = []
end

gam = kv + hw * kp;
num = [kat, kv, kp];            % kat s^2 + kv s + kp
den = [tau, 1, gam, kp];        % tau s^3 + s^2 + gamma s + kp

if nargout > 2
    s = 1i * omega;
    H = polyval(num, s) ./ polyval(den, s);
end
end
