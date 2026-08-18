function S = figs_time_domain(outDir, noiseMode)
%FIGS_TIME_DOMAIN  Reproduce Figs. 7, 8, 10, 12, 13, 14, 15 of Ma et al. (2025).
%
%   S = FIGS_TIME_DOMAIN(outDir, noiseMode)
%
%   noiseMode defaults to 'mean', i.e. the EQUIVALENT DETERMINISTIC SYSTEM of
%   eq. (13) / Remark 5, in which w_{i,i-1}(t) is replaced by its expectation
%   E[w] = 1.0482 and k_a_tilde = k_a E[w].  That is the system the paper's
%   robust string-stability analysis is performed on, and it is what
%   reproduces the smooth, monotone max|delta_i| profiles of Figs. 7, 10, 12.
%   Running with 'stochastic' instead adds realisation jitter of roughly
%   +-2 %, which is larger than the 0.4 %-per-hop trend being displayed --
%   the trend is a property of the MEAN system, not of any single sample path
%   (this is exactly the point of Remark 5).
%
%   Fig. 8 is always generated from a STOCHASTIC run, because its whole
%   content is the realised noise scatter inside the admissible cone.
%
%   Returns a struct S with the per-figure numbers for the results table.

arguments
    outDir    (1,:) char = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results')
    noiseMode (1,:) char = 'mean'
end
if ~isfolder(outDir), mkdir(outDir); end

% ======================================================================
% Fig. 7 -- Case 1 spacing errors
% ======================================================================
P1 = ma2025.params('case1', 'noise_mode', noiseMode);
R1 = ma2025.run_case(P1);
S.case1 = deltaFigure(R1, 'Fig 7 - Case 1 spacing errors', 'fig07_case1_delta.png', ...
    'Fig. 7  Case 1  (k_a = 0.5, h_w = 0.95 s, k_v = 0.63, k_p = 0.009)', outDir);
report('Fig 7  Case 1', S.case1, [1.593 1.535]);

% ======================================================================
% Fig. 8 -- noise and communicated signal on link 1 -> 2  (stochastic)
% ======================================================================
P8 = ma2025.params('case1', 'noise_mode', 'stochastic');
R8 = ma2025.run_case(P8);
S.fig8 = noiseFigure(R8, outDir);

% ======================================================================
% Fig. 10 -- Case 2 spacing errors (string unstable)
% ======================================================================
P2 = ma2025.params('case2', 'noise_mode', noiseMode);
R2 = ma2025.run_case(P2);
S.case2 = deltaFigure(R2, 'Fig 10 - Case 2 spacing errors', 'fig10_case2_delta.png', ...
    'Fig. 10  Case 2  (k_a = 0.5, h_w = 0.65 s, k_v = 0.63, k_p = 0.009)', outDir);
report('Fig 10 Case 2', S.case2, [0.882 0.886]);

% ======================================================================
% Fig. 12 -- Case 3 spacing errors (optimal gain)
% ======================================================================
P3 = ma2025.params('case3', 'noise_mode', noiseMode);
R3 = ma2025.run_case(P3);
S.case3 = deltaFigure(R3, 'Fig 12 - Case 3 spacing errors', 'fig12_case3_delta.png', ...
    'Fig. 12  Case 3  (k_a = k_a^*, h_w = 0.88 s, k_v = 0.85, k_p = 0.003)', outDir);
report('Fig 12 Case 3', S.case3, [0.802 0.798]);

% ======================================================================
% Fig. 13 -- Case 4, total platoon length for two headways
% ======================================================================
Pa = ma2025.params('case4a', 'noise_mode', noiseMode);  Ra = ma2025.run_case(Pa);
Pb = ma2025.params('case4b', 'noise_mode', noiseMode);  Rb = ma2025.run_case(Pb);

fh = ma2025.figstyle('Fig 13 - platoon length');
plot(Ra.t, Ra.len, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on
plot(Rb.t, Rb.len, ':', 'Color', [0.00 0.62 0.45], 'LineWidth', 1.8);
xlabel('t  (s)'); ylabel('(x_0 - x_N)  (m)');
title('Fig. 13  Case 4: total platoon length vs time headway');
legend({sprintf('h_w = %.2f s', Pa.hw), sprintf('h_w = %.2f s', Pb.hw)}, ...
       'Location', 'northeast');
xlim([0 200]);
exportgraphics(fh, fullfile(outDir, 'fig13_platoon_length.png'), 'Resolution', 200);

S.len = struct('hw_long', Pa.hw, 'hw_short', Pb.hw, ...
    'start_long', Ra.len(1), 'start_short', Rb.len(1), ...
    'peak_long', max(Ra.len), 'peak_short', max(Rb.len));
fprintf('Fig 13 Case 4: length at t=0  %.2f m (h=%.2f) vs %.2f m (h=%.2f)\n', ...
        Ra.len(1), Pa.hw, Rb.len(1), Pb.hw);
fprintf('               peak length    %.2f m         vs %.2f m   [paper ~437 / ~418]\n', ...
        max(Ra.len), max(Rb.len));
fprintf('               throughput gain from the shorter headway: %.2f m/vehicle\n', ...
        (Ra.len(1) - Rb.len(1)) / Pa.N);

% ======================================================================
% Figs. 14, 15 -- Case 5, heterogeneous parasitic lags
% ======================================================================
P5 = ma2025.params('case5', 'noise_mode', noiseMode);
R5 = ma2025.run_case(P5);
S.case5 = deltaFigure(R5, 'Fig 14 - Case 5 spacing errors', 'fig14_case5_delta.png', ...
    'Fig. 14  Case 5  (heterogeneous \tau_i, Table II; Case 1 gains)', outDir);

fh = ma2025.figstyle('Fig 15 - Case 5 accelerations', 620, 560);
cmap = parula(P5.N);
subplot(2,1,1);
hold on; for i = 1:P5.N, plot(R5.t, R5.a(:,i), 'Color', cmap(i,:)); end
xlabel('t  (s)'); ylabel('a_i  (m/s^2)'); xlim([0 200]); grid on; box on
title('Fig. 15  Case 5: follower accelerations');
subplot(2,1,2);
plot(1:P5.N, R5.maxAbsAccel, '-o', 'MarkerFaceColor', [0.00 0.62 0.45], ...
     'Color', [0.5 0.5 0.5]);
xlabel('Vehicle index'); ylabel('max |a_i|  (m/s^2)'); grid on; box on
exportgraphics(fh, fullfile(outDir, 'fig15_case5_accel.png'), 'Resolution', 200);

S.case5.maxAbsAccel = R5.maxAbsAccel;
S.case5.taus = P5.tau_vec(:).';
fprintf('Fig 14 Case 5: max|delta| %.4f .. %.4f (zig-zag from heterogeneous tau_i)\n', ...
        min(R5.maxAbsDelta), max(R5.maxAbsDelta));
fprintf('Fig 15 Case 5: max|a_i| %.4f -> %.4f  [paper ~0.49 -> ~0.455]\n', ...
        R5.maxAbsAccel(1), R5.maxAbsAccel(end));

S.noiseMode = noiseMode;
end

% ======================================================================
function out = deltaFigure(R, figName, fileName, ttl, outDir)
N = R.P.N;
fh = ma2025.figstyle(figName, 620, 560);
cmap = parula(N);

subplot(2,1,1); hold on
for i = 1:N, plot(R.t, R.delta(:,i), 'Color', cmap(i,:)); end
xlabel('t  (s)'); ylabel('\delta_i  (m)'); xlim([0 200]); grid on; box on
title(ttl, 'FontSize', 10);

subplot(2,1,2);
plot(1:N, R.maxAbsDelta, '-o', 'MarkerFaceColor', [0.00 0.62 0.45], ...
     'Color', [0.5 0.5 0.5]);
xlabel('Vehicle index'); ylabel('max |\delta_i|  (m)'); grid on; box on
exportgraphics(fh, fullfile(outDir, fileName), 'Resolution', 200);

out.maxAbsDelta = R.maxAbsDelta;
out.first = R.maxAbsDelta(1);
out.last  = R.maxAbsDelta(end);
out.trend = sign(R.maxAbsDelta(end) - R.maxAbsDelta(1));
out.monotone = all(diff(R.maxAbsDelta) < 0) || all(diff(R.maxAbsDelta) > 0);
end

% ======================================================================
function out = noiseFigure(R, outDir)
%  Link 1 -> 2 : the receiver is follower 2, so the factor is w(:,2) and the
%  ideal signal is a_1.  Plot exactly the paper's two panels.
a1 = R.a(:,1);
w21 = R.w(:,2);
rho = R.P.rho;
if numel(w21) ~= numel(a1)
    % w is logged on the discrete channel rate; put it on the state grid.
    w21 = interp1(linspace(R.t(1), R.t(end), numel(w21)), w21, R.t, ...
                  'previous', 'extrap');       % zero-order hold, as realised
end
noise = (w21 - 1) .* a1;               % n(t) = w a1 - a1

ax = linspace(min(a1), max(a1), 200);

fh = ma2025.figstyle('Fig 8 - noise on link 1->2', 620, 560);
subplot(2,1,1);
plot(a1, noise, '.', 'Color', [0.30 0.75 0.93], 'MarkerSize', 3); hold on
plot(ax, -ax/rho, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
plot(ax,  ax/rho, 'Color', [0.00 0.62 0.45], 'LineWidth', 1.3);
xlabel('a_1  (m/s^2)'); ylabel('Noise'); grid on; box on
legend({'realised n(t)', '-a_1/\rho', '+a_1/\rho'}, 'Location', 'northwest', 'FontSize', 9);
title('Fig. 8  Case 1: noise and communicated acceleration, link 1 \rightarrow 2', ...
      'FontSize', 10);

subplot(2,1,2);
plot(a1, w21 .* a1, '.', 'Color', [0.93 0.85 0.20], 'MarkerSize', 3); hold on
plot(ax, (1 - 1/rho)*ax, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
plot(ax, (1 + 1/rho)*ax, 'Color', [0.00 0.62 0.45], 'LineWidth', 1.3);
xlabel('a_1  (m/s^2)'); ylabel('w_{2,1} a_1'); grid on; box on
legend({'realised', '(1-1/\rho) a_1', '(1+1/\rho) a_1'}, 'Location', 'northwest', 'FontSize', 9);
exportgraphics(fh, fullfile(outDir, 'fig08_noise_link12.png'), 'Resolution', 200);

out.w_min = min(w21);  out.w_max = max(w21);  out.w_mean = mean(w21);
out.bound_lo = 1 - 1/rho;  out.bound_hi = 1 + 1/rho;
out.inside = all(w21 >= out.bound_lo - 1e-12) && all(w21 < out.bound_hi + 1e-12);
fprintf('Fig 8  channel realisation on link 1->2: w in [%.4f, %.4f], mean %.4f\n', ...
        out.w_min, out.w_max, out.w_mean);
fprintf('       admissible support [%.1f, %.1f) respected: %d   (E[w] = %.4f)\n', ...
        out.bound_lo, out.bound_hi, out.inside, ma2025.expected_w(rho));
end

% ======================================================================
function report(tag, s, paperVals)
fprintf('%s: max|delta| %.4f -> %.4f  (%s, monotone %d)  [paper %.3f -> %.3f]\n', ...
        tag, s.first, s.last, ternary(s.trend < 0, 'decreasing', 'INCREASING'), ...
        s.monotone, paperVals(1), paperVals(2));
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
