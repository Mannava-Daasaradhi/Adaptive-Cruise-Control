function figs_design_space(outDir)
%FIGS_DESIGN_SPACE  Reproduce Figs. 4, 5 and 11 of Ma et al. (2025).
%
%   FIGS_DESIGN_SPACE(outDir)
%
%   Fig. 4  f(ka) = -m n ka^2 + 2 n ka - m  vs ka   (rho = 5)
%           The numerator of d h_lb/d ka.  Its smaller root r1 IS ka*, and
%           the fact that r1 < 1/(1+1/rho) < r2 is what makes the minimiser
%           interior to the admissible interval -- Statement (c) of Thm 2.
%
%   Fig. 5  Case 1 feasible region of (kv, kp) for ka = 0.5, hw = 0.95 s.
%   Fig. 11 Case 3 feasible region for ka = ka*, hw = 0.88 s.
%           Both are the wedge S = S1 n S2 of eq. (35): a straight upper
%           edge from (28a) and a straight lower edge from (28b).  The lower
%           edge is straight only AFTER squaring (28b) -- see (32)->(33) --
%           which is why the region is a polygon and not a conic slice.
%
%   Also overlays the paper's chosen gains and prints where they sit.

arguments
    outDir (1,:) char = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results')
end
if ~isfolder(outDir), mkdir(outDir); end

rho  = 5;
tau0 = 0.5;

% ======================================================================
% Fig. 4 -- f(ka)
% ======================================================================
m = 1 - 1/rho;
n = (1 + 1/rho)^2;
ka = linspace(0, 2.5, 1200);
f  = -m*n*ka.^2 + 2*n*ka - m;

r1 = ma2025.ka_star(rho);
r2 = (n + sqrt(n*(n - m^2))) / (m*n);
kaMax = 1/(1 + 1/rho);

fh = ma2025.figstyle('Fig 4 - f(ka)');
plot(ka, f, 'LineWidth', 1.6); hold on
yline(0, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
xline(kaMax, '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.2, ...
      'Label', '1/(1+1/\rho)', 'LabelOrientation', 'horizontal');
plot(r1, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
plot(r2, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
text(r1, 0.12, sprintf('  r_1 = k_a^* = %.4f', r1), 'FontSize', 10);
text(r2 - 0.55, 0.12, sprintf('r_2 = %.4f  ', r2), 'FontSize', 10);
xlabel('k_a'); ylabel('f(k_a)');
title(sprintf('Fig. 4  f(k_a) vs k_a,  \\rho = %g', rho));
xlim([0 2.5]); ylim([-0.85 1.05]);
exportgraphics(fh, fullfile(outDir, 'fig04_f_of_ka.png'), 'Resolution', 200);

fprintf('Fig 4 : r1 = ka* = %.6f  (paper 0.3183),  r2 = %.6f,  1/(1+1/rho) = %.4f\n', ...
        r1, r2, kaMax);
fprintf('        f decreasing-then-increasing => h_lb minimised at r1 (eq. 38)\n');

% ======================================================================
% Figs. 5 and 11 -- feasible regions
% ======================================================================
plotRegion(0.5,              0.95, 0.63, 0.009, ...
    'Fig 5 - Case 1 feasible region',  'fig05_feasible_case1.png', ...
    [0.615 0.650], [-0.010 0.025], outDir, rho, tau0, 'Fig. 5  Case 1:  k_a = 0.5,  h_w = 0.95 s');

plotRegion(ma2025.ka_star(rho), 0.88, 0.85, 0.003, ...
    'Fig 11 - Case 3 feasible region', 'fig11_feasible_case3.png', ...
    [0.830 0.860], [-0.020 0.030], outDir, rho, tau0, 'Fig. 11  Case 3:  k_a = k_a^*,  h_w = 0.88 s');
end

% ======================================================================
function plotRegion(ka, hw, kvPick, kpPick, figName, fileName, xl, yl, outDir, rho, tau0, ttl)
F = ma2025.feasible_set(ka, rho, hw, tau0);

kv = linspace(xl(1), xl(2), 800);
kpUpper = F.b1 * (1 - kv / F.a1);      % (30)/(31): kv/a1 + kp/b1 = 1
kpLower = F.b2 * (1 - kv / F.a2);      % (33)/(34): kv/a2 + kp/b2 = 1

fh = ma2025.figstyle(figName);
% shade S = S1 n S2 n {kp > 0}
lo = max(kpLower, 0);
hi = kpUpper;
ok = hi > lo;
patch([kv(ok), fliplr(kv(ok))], [lo(ok), fliplr(hi(ok))], ...
      [0.93 0.69 0.13], 'FaceAlpha', 0.28, 'EdgeColor', 'none'); hold on
plot(kv, kpUpper, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
plot(kv, kpLower, 'Color', [0.00 0.62 0.45], 'LineWidth', 1.5);
yline(0, 'Color', [0 0.6 0.6], 'LineWidth', 1.1);
plot(kvPick, kpPick, 'kp', 'MarkerFaceColor', 'k', 'MarkerSize', 11);
text(kvPick, kpPick, sprintf('  (%.3f, %.4f)', kvPick, kpPick), 'FontSize', 10);

xlabel('k_v'); ylabel('k_p'); title(ttl);
xlim(xl); ylim(yl);
legend({'S = S_1 \cap S_2', 'upper edge, eq. (28a)', 'lower edge, eq. (28b)'}, ...
       'Location', 'southwest', 'FontSize', 9);
exportgraphics(fh, fullfile(outDir, fileName), 'Resolution', 200);

C = ma2025.check_gains(ka, rho, kvPick, kpPick, hw, tau0);
fprintf('%s\n', ttl);
fprintf('   h_lb = %.4f s  (hw = %.4f)   gamma = %.5f  in [%.5f, %.5f]\n', ...
        F.h_lb, hw, C.gamma, C.gamma_min, C.gamma_max);
fprintf('   kv-intercepts: upper %.4f  lower %.4f      a1/a2 = %.4f (> 1 => S nonempty)\n', ...
        F.a1, F.a2, F.ratio_a1a2);
fprintf('   picked gains admissible: eq16 %d eq17 %d eq28a %d eq28b %d RH %d | ||H||inf(worst) = %.6f\n', ...
        C.eq16_ok, C.eq17_ok, C.eq28a_ok, C.eq28b_ok, C.rh_ok, C.hinf_worst);
end
