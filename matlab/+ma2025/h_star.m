function h = h_star(rho, tau0)
%H_STAR  Minimum robust time headway, Theorem 2(c), eqs. (19)/(42).
%
%   h = MA2025.H_STAR(rho, tau0)
%
%                        (1 + 1/sqrt(rho))^2
%       h*_{w,lb} = tau0 --------------------                            (19)
%                            1 + 1/rho
%
%   i.e. h_lb evaluated at ka = ka*.  Equals 2*tau0*hbar(r1) with
%   hbar(r1) = (1 + beta)^2 / (2(1 + beta^2)), beta = 1/sqrt(rho)  (41).
%
%   Monotone decreasing in rho: a cleaner channel buys a shorter headway,
%   and h* -> tau0 as rho -> Inf (Remark 4) -- the noiseless floor.
%
%   See also MA2025.KA_STAR, MA2025.H_LB.

arguments
    rho  (1,:) double {mustBeGreaterThan(rho,1)}
    tau0 (1,1) double {mustBePositive} = 0.5
end
rs = 1 ./ sqrt(rho);
h  = tau0 * (1 + rs).^2 ./ (1 + 1 ./ rho);
end
