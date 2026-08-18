function run_all(noiseMode)
%RUN_ALL  Full reproduction of Ma, Pagilla & Darbha (IEEE T-ITS 2025) in Simulink.
%
%   run_all              % deterministic-equivalent channel (paper's Remark 5)
%   run_all('stochastic')% realised 16-bit Bernoulli channel, eq. (5)
%
%   Builds the model if needed, verifies every closed form of Theorems 1-2,
%   and regenerates Figs. 4-15 into matlab/results/ together with a transcript
%   in results/run_all_log.txt.
%
%   Wall time: about 1-2 minutes (7 platoon simulations of 200 s at 1 ms).

arguments
    noiseMode (1,:) char = 'mean'
end

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'models'), fullfile(here, 'scripts'));
outDir = fullfile(here, 'results');
if ~isfolder(outDir), mkdir(outDir); end

logFile = fullfile(outDir, 'run_all_log.txt');
if isfile(logFile), delete(logFile); end
diary(logFile); diary on
cleanup = onCleanup(@() diary('off'));

fprintf('========================================================\n');
fprintf(' Ma, Pagilla & Darbha, IEEE T-ITS 26(1):1029-1038, 2025\n');
fprintf(' Simulink reproduction  |  channel mode: %s\n', noiseMode);
fprintf('========================================================\n');

% ---- 1. model ---------------------------------------------------------
mdlFile = fullfile(here, 'models', 'cacc_platoon.slx');
if ~isfile(mdlFile)
    build_cacc_model('cacc_platoon', fullfile(here, 'models'));
else
    fprintf('\nmodel: %s (already built; delete to force a rebuild)\n', mdlFile);
end

% ---- 2. theory --------------------------------------------------------
T = theorem_checks();  %#ok<NASGU>

% ---- 3. design-space figures -----------------------------------------
fprintf('\n=== Figs. 4, 5, 11 : design space ===\n');
figs_design_space(outDir);

% ---- 4. frequency-domain figures -------------------------------------
fprintf('\n=== Figs. 6, 9 : frequency response ===\n');
figs_frequency(outDir);

% ---- 5. time-domain figures ------------------------------------------
fprintf('\n=== Figs. 7, 8, 10, 12, 13, 14, 15 : simulations ===\n');
S = figs_time_domain(outDir, noiseMode);

% ---- 6. summary -------------------------------------------------------
fprintf('\n=== Reproduction summary ===\n');
fprintf('%-24s %10s %10s %10s\n', 'quantity', 'paper', 'ours', 'rel.err');
cmp('h_w,lb (ka=0.5)',   0.9375, ma2025.h_lb(0.5,5,0.5));
cmp('ka*',               0.3183, ma2025.ka_star(5));
cmp('h*_w,lb',           0.8727, ma2025.h_star(5,0.5));
cmp('ka upper bound',    0.8333, 1/(1+1/5));
cmp('Case 1 max|d_1|',   1.593,  S.case1.first);
cmp('Case 1 max|d_12|',  1.535,  S.case1.last);
cmp('Case 2 max|d_1|',   0.882,  S.case2.first);
cmp('Case 2 max|d_12|',  0.886,  S.case2.last);
cmp('Case 3 max|d_1|',   0.802,  S.case3.first);
cmp('Case 3 max|d_12|',  0.798,  S.case3.last);
cmp('Case 4 len h=0.95', 345.0,  S.len.start_long);
cmp('Case 4 len h=0.88', 324.0,  S.len.start_short);

fprintf('\nString-stability verdicts (the claims that matter):\n');
fprintf('  Case 1 max|delta_i| %s along the string  -> attenuating\n', ...
        arrow(S.case1.trend));
fprintf('  Case 2 max|delta_i| %s along the string  -> AMPLIFYING (unstable)\n', ...
        arrow(S.case2.trend));
fprintf('  Case 3 max|delta_i| %s along the string  -> attenuating\n', ...
        arrow(S.case3.trend));

fprintf('\nfigures written to %s\n', outDir);
diary off
end

% ======================================================================
function cmp(name, paper, ours)
rel = abs(ours - paper) / max(abs(paper), eps);
flag = '';
if rel > 0.02, flag = '   <-- see reproduction note'; end
fprintf('%-24s %10.4f %10.4f %9.2f%%%s\n', name, paper, ours, 100*rel, flag);
end

function s = arrow(tr)
if tr < 0, s = 'DECREASES'; else, s = 'INCREASES'; end
end
