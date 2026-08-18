function ka = ka_star(rho)
%KA_STAR  Headway-minimising acceleration gain, Theorem 2(c), eqs. (18)/(40).
%
%   ka = MA2025.KA_STAR(rho)
%
%                 1 - 1/sqrt(rho)        1
%       ka* =  ------------------- * ----------                          (18)
%                 1 + 1/sqrt(rho)     1 + 1/rho
%
%   Obtained as the smaller root r1 of f(ka) = -m n ka^2 + 2 n ka - m, the
%   numerator of d h_lb/d ka, with m = 1 - 1/rho and n = (1 + 1/rho)^2:
%
%       r1 = ( n - sqrt(n(n - m^2)) ) / (m n)                            (40)
%
%   which simplifies (n - m^2 = 4/rho) to the closed form above.  h_lb is
%   decreasing on (0, r1) and increasing on (r1, 1/(1+1/rho)), so r1 is the
%   global minimiser on the admissible interval (eq. 38).
%
%   As rho -> Inf, ka* -> 1 (Remark 4): with a clean channel you want as
%   much feedforward as internal stability allows.
%
%   See also MA2025.H_STAR, MA2025.H_LB.

arguments
    rho (1,:) double {mustBeGreaterThan(rho,1)}
end
rs = 1 ./ sqrt(rho);
ka = ((1 - rs) ./ (1 + rs)) ./ (1 + 1 ./ rho);
end
