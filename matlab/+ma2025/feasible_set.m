function F = feasible_set(ka, rho, hw, tau0)
%FEASIBLE_SET  The sets S1, S2 and S = S1 n S2 of Theorem 2, eqs. (30)-(35).
%
%   F = MA2025.FEASIBLE_SET(ka, rho, hw, tau0) returns a struct with the
%   intercepts that define the two half-planes in the (kv, kp) plane, ready
%   for plotting the paper's Figs. 5 and 11.
%
%   UPPER boundary -- from (28a), the high-noise-end / high-frequency
%   condition gamma <= (1 - (1+1/rho)^2 ka^2)/(2 tau0).  With gamma = kv +
%   hw kp this is the straight line (30):
%
%       kv/a1 + kp/b1 <= 1 ,   a1 = (1 - (1+1/rho)^2 ka^2)/(2 tau0)
%                              b1 = a1 / hw                              (31)
%
%   LOWER boundary -- from (28b), the low-noise-end / low-frequency
%   condition gamma^2 >= 2 kp (1 - (1-1/rho) ka) + kv^2.  Squaring and
%   cancelling kv^2 leaves 2 hw kv + hw^2 kp >= 2(1 - (1-1/rho) ka)  (33),
%   i.e. ANOTHER straight line -- the quadratic collapses:
%
%       kv/a2 + kp/b2 >= 1 ,   a2 = (1 - (1-1/rho) ka)/hw
%                              b2 = 2 a2 / hw                            (34)
%
%   S := S1 n S2 (35) is the wedge between them (plus kv, kp > 0).  It is
%   non-empty iff a1 >= a2 or b1 >= b2; substituting hw > h_lb from (17)
%   makes a1/a2 > 1, which is precisely how Theorem 2(b) is proved.
%
%   Fields: a1 b1 a2 b2 (intercepts), ratio_a1a2, nonempty, plus the
%   Routh-Hurwitz internal-stability requirement gamma > tau0 kp.
%
%   See also MA2025.CHECK_GAINS, MA2025.H_LB.

arguments
    ka   (1,1) double
    rho  (1,1) double
    hw   (1,1) double
    tau0 (1,1) double = 0.5
end

m = 1 - 1/rho;                  % smallest admissible channel gain factor
n = (1 + 1/rho)^2;              % squared largest admissible factor

upperNum = 1 - n * ka^2;        % 1 - (1+1/rho)^2 ka^2   -- must be > 0
lowerNum = 1 - m * ka;          % 1 - (1-1/rho) ka

F.ka = ka;  F.rho = rho;  F.hw = hw;  F.tau0 = tau0;
F.a1 = upperNum / (2 * tau0);
F.b1 = upperNum / (2 * tau0 * hw);
F.a2 = lowerNum / hw;
F.b2 = 2 * lowerNum / hw^2;

F.gamma_max = upperNum / (2 * tau0);            % (28a) ceiling on gamma
F.h_lb      = ma2025.h_lb(ka, rho, tau0);       % (17)
F.ratio_a1a2 = (hw / (2 * tau0)) * upperNum / lowerNum;   % = a1/a2
F.nonempty   = (F.a1 >= F.a2) || (F.b1 >= F.b2);
end
