function T = theorem_checks()
%THEOREM_CHECKS  Numerically verify every closed-form claim of Ma et al. (2025).
%
%   T = THEOREM_CHECKS()
%
%   Each block below is a claim in the paper checked against an INDEPENDENT
%   numerical route, so agreement means the algebra and the code agree rather
%   than the code merely re-stating itself:
%
%     A  printed numbers (eqs. 18, 19, 44, 45)      -> closed form vs paper
%     B  eq. (25) polynomial identity                -> symbolic-free residual
%                                                       of |D|^2 - |N|^2
%     C  Theorem 2(c) optimality of ka*              -> closed form vs a fine
%                                                       grid search on h_lb
%     D  Remark 4 limits as rho -> Inf               -> h_lb -> 2 tau0/(1+ka),
%                                                       ka* -> 1, h* -> tau0
%     E  Theorem 1(b) vs Theorem 2(b)                -> 2 tau0/(1+kat) <= h_lb
%     F  (28) is SUFFICIENT, not NECESSARY           -> exhibit a design that
%                                                       fails (28) yet has
%                                                       ||H||inf <= 1
%     G  Routh-Hurwitz on D(s)                       -> closed form vs roots()
%
%   Prints a pass/fail table and returns the numbers.

rho = 5;  tau0 = 0.5;
tol = 1e-9;
T = struct();
pass = @(c) ternary(c, 'PASS', 'FAIL');

fprintf('\n=== A. Printed numbers, Sec. IV ===\n');
T.ka_max  = 1/(1 + 1/rho);
T.h_lb    = ma2025.h_lb(0.5, rho, tau0);
T.ka_star = ma2025.ka_star(rho);
T.h_star  = ma2025.h_star(rho, tau0);
T.Ew      = ma2025.expected_w(rho);
row('eq (44)  ka < 1/(1+1/rho)', T.ka_max,  0.8333, 1e-4);
row('eq (45)  h_w,lb(0.5)',      T.h_lb,    0.9375, 1e-9);
row('eq (18)  ka*',              T.ka_star, 0.3183, 1e-4);
row('eq (19)  h*_w,lb',          T.h_star,  0.8727, 1e-4);
fprintf('  %-28s %12.6f   (Table I, not printed in the paper)\n', 'eq (12)  E[w]', T.Ew);

fprintf('\n=== B. eq. (25) is |D(jw)|^2 - |N(jw)|^2 divided by w^2 ===\n');
% Claim: |D|^2 - |N|^2 = w^2 * [ tau^2 w^4 + (1 - kat^2 - 2 tau gam) w^2
%                                + gam^2 - 2 kp - kv^2 + 2 kat kp ]
kat = 0.5*T.Ew;  kv = 0.63;  kp = 0.009;  hw = 0.95;  gam = kv + hw*kp;
w  = linspace(0.01, 5, 4000);
[~, ~, H] = ma2025.Htilde(kat, kv, kp, hw, tau0, w);
lhs = abs(polyval([tau0 1 gam kp], 1i*w)).^2 - abs(polyval([kat kv kp], 1i*w)).^2;
rhs = w.^2 .* (tau0^2*w.^4 + (1 - kat^2 - 2*tau0*gam)*w.^2 ...
               + gam^2 - 2*kp - kv^2 + 2*kat*kp);
T.eq25_residual = max(abs(lhs - rhs));
fprintf('  max |LHS - RHS| over w in [0.01, 5] = %.3e   %s\n', ...
        T.eq25_residual, pass(T.eq25_residual < 1e-9));
% and the sign of that expression must match |H| <= 1
agree = all((lhs >= -1e-12) == (abs(H) <= 1 + 1e-12));
fprintf('  sign(|D|^2-|N|^2) matches (|H| <= 1) pointwise: %s\n', pass(agree));
T.eq25_sign_agrees = agree;

fprintf('\n=== C. ka* really minimises h_lb (Theorem 2c) ===\n');
kaGrid = linspace(1e-6, T.ka_max - 1e-6, 400001);
hGrid  = ma2025.h_lb(kaGrid, rho, tau0);
[hMin, ix] = min(hGrid);
T.ka_star_grid = kaGrid(ix);  T.h_star_grid = hMin;
row('ka* grid vs closed form', T.ka_star_grid, T.ka_star, 1e-5);
row('h*  grid vs closed form', T.h_star_grid,  T.h_star,  1e-9);

fprintf('\n=== D. Remark 4: rho -> Inf recovers the noiseless design ===\n');
big = 1e12;  ka = 0.5;
T.hlb_limit  = ma2025.h_lb(ka, big, tau0);
T.hlb_ideal  = 2*tau0/(1 + ka);
row('h_lb(rho=1e12) vs 2 tau0/(1+ka)', T.hlb_limit, T.hlb_ideal, 1e-6);
T.kastar_limit = ma2025.ka_star(big);
T.hstar_limit  = ma2025.h_star(big, tau0);
row('ka*(rho -> Inf) -> 1',   T.kastar_limit, 1.0,  1e-5);
row('h* (rho -> Inf) -> tau0', T.hstar_limit, tau0, 1e-5);

fprintf('\n=== E. Theorem 1(b) bound is weaker than Theorem 2(b) ===\n');
% 2 tau0/(1 + kat) with the WORST (smallest) admissible kat must be <= h_lb.
katLow = (1 - 1/rho)*ka;
T.thm1_bound = 2*tau0/(1 + katLow);
fprintf('  2 tau0/(1 + (1-1/rho) ka) = %.6f   h_lb = %.6f   ordering %s\n', ...
        T.thm1_bound, T.h_lb, pass(T.thm1_bound <= T.h_lb + tol));
fprintf('  (Theorem 2 adds the cost of making (kv,kp) simultaneously feasible\n');
fprintf('   for EVERY kat in I, which is the (1+1/rho)^2 ka^2 denominator.)\n');
T.thm1_le_thm2 = T.thm1_bound <= T.h_lb + tol;

fprintf('\n=== F. (28a)/(28b) are sufficient, not necessary ===\n');
% Search for a design that violates the sufficient conditions but is still
% string stable by direct evaluation of ||H||inf over the whole interval.
found = [];
for hwT = [1.5 2.0 3.0 5.0]
    for kvT = [0.70 0.80 0.90]
        C = ma2025.check_gains(0.5, rho, kvT, 0.009, hwT, tau0);
        if ~C.sufficient_ok && C.hinf_ok
            found = C; break
        end
    end
    if ~isempty(found), break; end
end
if isempty(found)
    fprintf('  no counterexample in the probed box (conditions tight here)\n');
    T.sufficiency_gap = [];
else
    fprintf('  counterexample: ka=0.5 kv=%.2f kp=0.009 hw=%.2f\n', found.kv, found.hw);
    fprintf('    eq28a %d  eq28b %d  -> sufficient conditions FAIL\n', ...
            found.eq28a_ok, found.eq28b_ok);
    fprintf('    ||H||inf worst over I = %.6f <= 1  -> ACTUALLY STRING STABLE  %s\n', ...
            found.hinf_worst, pass(true));
    fprintf('  Why: (26) demands each power of w^2 be non-negative separately;\n');
    fprintf('  a positive tau^2 w^6 term can offset a slightly negative w^4 term.\n');
    T.sufficiency_gap = found;
end

fprintf('\n=== G. Routh-Hurwitz on D(s) = tau s^3 + s^2 + gamma s + kp ===\n');
% RH for a cubic a3 s^3 + a2 s^2 + a1 s + a0: all ai > 0 and a2 a1 > a3 a0,
% i.e. gamma > tau kp.  Compare with the actual roots.
cases = {0.5 0.63 0.009 0.95; 0.5 0.63 0.009 0.65; ma2025.ka_star(rho) 0.85 0.003 0.88};
T.rh = zeros(size(cases,1), 3);
for k = 1:size(cases,1)
    kvk = cases{k,2}; kpk = cases{k,3}; hwk = cases{k,4};
    gk = kvk + hwk*kpk;
    r  = roots([tau0 1 gk kpk]);
    T.rh(k,:) = [gk - tau0*kpk, max(real(r)), double(gk > tau0*kpk)];
    fprintf('  hw=%.2f: gamma - tau0 kp = %+.5f   max Re(pole) = %+.5f   %s\n', ...
            hwk, gk - tau0*kpk, max(real(r)), pass((gk > tau0*kpk) == (max(real(r)) < 0)));
end

fprintf('\n');
end

% ======================================================================
function row(name, got, want, tol)
ok = abs(got - want) <= tol;
fprintf('  %-28s %12.6f   paper %10.4f   %s\n', name, got, want, ...
        ternary(ok, 'PASS', 'FAIL'));
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
