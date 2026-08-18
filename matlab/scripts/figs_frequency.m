function figs_frequency(outDir)
%FIGS_FREQUENCY  Reproduce Figs. 6 and 9 of Ma et al. (2025).
%
%   FIGS_FREQUENCY(outDir)
%
%   Both figures plot the ENVELOPE of |Htilde(j omega; tau0)| as the effective
%   feedforward gain k_a_tilde sweeps the whole admissible noise interval
%
%       I = [(1 - 1/rho) k_a , (1 + 1/rho) k_a] = [0.4, 0.6]   for k_a = 0.5
%
%   i.e. max and min over I at each frequency, with the band shaded.  This is
%   the picture that actually decides string stability: Definition 2 asks for
%   ||Htilde||_inf <= 1 for EVERY admissible channel realisation, so the red
%   (max) curve is the verdict and the green (min) curve is only context.
%
%   Fig. 6  Case 1 (hw = 0.95 s): the max curve stays at or below 1  -> stable.
%   Fig. 9  Case 2 (hw = 0.65 s): the max curve exceeds 1 at low frequency
%           -> string UNstable, and the violation is at the LOW end of I.
%
%   Note |Htilde(0)| = kp/kp = 1 identically, so every curve starts at 1: the
%   question is only whether it leaves 1 upward or downward.  This is why the
%   interesting behaviour is confined to a decade or so above DC and the
%   paper plots a linear omega axis on [0, 1] rad/s.

arguments
    outDir (1,:) char = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results')
end
if ~isfolder(outDir), mkdir(outDir); end

rho = 5;  tau0 = 0.5;
omega = linspace(0, 1, 2000);

doOne('case1', 'Fig 6 - Case 1 frequency response', 'fig06_freq_case1.png', ...
      'Fig. 6  Case 1:  k_a = 0.5, h_w = 0.95 s, k_v = 0.63, k_p = 0.009', ...
      [0.70 1.01], outDir, rho, tau0, omega);

doOne('case2', 'Fig 9 - Case 2 frequency response', 'fig09_freq_case2.png', ...
      'Fig. 9  Case 2:  k_a = 0.5, h_w = 0.65 s, k_v = 0.63, k_p = 0.009', ...
      [0.70 1.01], outDir, rho, tau0, omega);
end

% ======================================================================
function doOne(caseName, figName, fileName, ttl, yl, outDir, rho, tau0, omega)
P = ma2025.params(caseName);
I = ma2025.ka_tilde(P.ka, rho, 'interval');
kats = linspace(I(1), I(2), 121);

M = zeros(numel(kats), numel(omega));
for k = 1:numel(kats)
    [~, ~, H] = ma2025.Htilde(kats(k), P.kv, P.kp, P.hw, tau0, omega);
    M(k,:) = abs(H);
end
hiC = max(M, [], 1);
loC = min(M, [], 1);

fh = ma2025.figstyle(figName);
patch([omega, fliplr(omega)], [loC, fliplr(hiC)], [0.93 0.69 0.13], ...
      'FaceAlpha', 0.25, 'EdgeColor', 'none'); hold on
plot(omega, hiC, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
plot(omega, loC, 'Color', [0.00 0.62 0.45], 'LineWidth', 1.5);
yline(1, 'k--', 'LineWidth', 1.0);
xlabel('\omega  (rad/s)'); ylabel('|H(j\omega; \tau_0)|');
title(ttl); ylim(yl); xlim([0 1]);
legend({'range over k_a\prime \in I', 'max_{I} |H|', 'min_{I} |H|', '1'}, ...
       'Location', 'southwest', 'FontSize', 9);
exportgraphics(fh, fullfile(outDir, fileName), 'Resolution', 200);

% ---- the numbers behind the picture -----------------------------------
peak = max(hiC);
[wv, ~] = max(hiC);  %#ok<ASGLU>
C = ma2025.check_gains(P.ka, rho, P.kv, P.kp, P.hw, tau0);
fprintf('%s\n', ttl);
fprintf('   I = [%.4f, %.4f];  sup over I and omega in [0,1] of |H| = %.6f\n', ...
        I(1), I(2), peak);
fprintf('   ||H||inf on the full grid: low-end %.6f | mean %.6f | high-end %.6f | WORST %.6f (at kat = %.4f)\n', ...
        C.hinf_low, C.hinf_mean, C.hinf_high, C.hinf_worst, C.kat_binding);
if C.hinf_worst <= 1 + 1e-9
    fprintf('   verdict: ROBUSTLY STRING STABLE\n');
else
    fprintf('   verdict: STRING UNSTABLE (violation %.3f%% above unity)\n', ...
            100*(C.hinf_worst - 1));
end
end
