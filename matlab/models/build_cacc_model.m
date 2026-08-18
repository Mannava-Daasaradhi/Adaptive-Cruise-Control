function mdl = build_cacc_model(mdl, savePath)
%BUILD_CACC_MODEL  Programmatically construct the CACC/CTHP platoon model.
%
%   mdl = BUILD_CACC_MODEL()
%   mdl = BUILD_CACC_MODEL(name, savePath)
%
%   Builds (and saves) cacc_platoon.slx, a Simulink realisation of the
%   noisy-V2V CTHP platoon of Ma, Pagilla & Darbha, IEEE T-ITS 26(1), 2025.
%   This .m file is the source of truth; the .slx is a build artefact, so the
%   model stays reviewable and diffable in git.
%
%   ---------------------------------------------------------------------
%   WHAT IS MODELLED
%   ---------------------------------------------------------------------
%   Plant, eq. (1), one per following vehicle i = 1..N:
%
%       xddot_i = a_i ,      tau_i adot_i + a_i = u_i
%
%   Spacing errors, Definition 1 and eq. (2):
%
%       e_i     = x_i - x_{i-1} + d      (paper's sign convention: e_i is
%                                         NEGATIVE of "gap minus d")
%       delta_i = e_i + hw v_i
%
%   CTHP control law, eq. (6):
%
%       u_i = ka w_{i,i-1}(t) a_{i-1} - kv (v_i - v_{i-1}) - kp delta_i
%
%   Multiplicative n-bit V2V channel, eq. (5):
%
%       w_{i,i-1}(t) = (1 - 1/rho) + (1/rho) sum_{j=0}^{15} z_{i,j}(t) 2^-j
%
%   ---------------------------------------------------------------------
%   WHY VECTORISED (width-N signals) RATHER THAN N COPIES OF A SUBSYSTEM
%   ---------------------------------------------------------------------
%   * N is a parameter, not a diagram edit: the same .slx runs the 12-car
%     paper case and any other platoon size.
%   * Heterogeneous parasitic lags (Case 5, Table II) are just a vector
%     tau_vec; nothing in the diagram changes.
%   * The predecessor shift, the ONLY coupling between vehicles, becomes one
%     constant matrix  PredSel = [eye(N) zeros(N,1)]  applied to the
%     augmented signal [leader ; followers].  That single block IS the
%     one-vehicle look-ahead information topology, visible at a glance.
%
%   NO ALGEBRAIC LOOP EXISTS: u_i depends on a_{i-1}, which is a STATE of
%   vehicle i-1 (the output of its lag integrator), never a direct
%   feedthrough of u_{i-1}.  Simulink therefore needs no artificial delay.
%
%   ---------------------------------------------------------------------
%   WORKSPACE VARIABLES THE MODEL RESOLVES (supplied by ma2025.run_case)
%   ---------------------------------------------------------------------
%   N ka kv kp hw d v_ss  tau_inv(Nx1)  PredSel(Nx(N+1))
%   noise_K(Nx16N) noise_bias(Nx1) gamma_rep(16Nx1) seed_vec(16Nx1) ts_noise
%   x_init(Nx1) v_init(Nx1) a_init(Nx1) x0_lead
%   a0_amp a0_omega a0_t0 a0_dur  dt Tend decim
%
%   See also MA2025.RUN_CASE, MA2025.PARAMS.

arguments
    mdl      (1,:) char = 'cacc_platoon'
    savePath (1,:) char = fileparts(mfilename('fullpath'))
end

load_system('simulink');
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);

% =======================================================================
% Top level
% =======================================================================
add_block('built-in/Subsystem', [mdl '/Lead Vehicle'], 'Position', [40 40 150 110]);
add_block('built-in/Subsystem', [mdl '/V2V Channel'],  'Position', [40 190 150 260]);
add_block('built-in/Subsystem', [mdl '/Platoon'],      'Position', [260 60 420 240]);

buildLeadVehicle([mdl '/Lead Vehicle']);
buildChannel([mdl '/V2V Channel']);
buildPlatoon([mdl '/Platoon']);

add_line(mdl, 'Lead Vehicle/1', 'Platoon/1', 'autorouting', 'on');
add_line(mdl, 'V2V Channel/1',  'Platoon/2', 'autorouting', 'on');

% ---- logging ----------------------------------------------------------
sigs = {'x', 'v', 'a', 'delta', 'u'};
for k = 1:numel(sigs)
    b = [mdl '/log_' sigs{k}];
    add_block('simulink/Sinks/To Workspace', b, ...
        'Position', [500, 40 + 60*(k-1), 580, 70 + 60*(k-1)], ...
        'VariableName', sigs{k}, 'SaveFormat', 'Timeseries', 'Decimation', 'decim');
    add_line(mdl, sprintf('Platoon/%d', k), ['log_' sigs{k} '/1'], 'autorouting', 'on');
end

% w lives on the DISCRETE ts_noise rate, not the ode4 step, so its decimation
% must be rescaled to land on the same log grid as the continuous signals:
%   log period = ts_noise * decim_w  ==  dt * decim
add_block('simulink/Sinks/To Workspace', [mdl '/log_w'], ...
    'Position', [500 340 580 370], 'VariableName', 'w', ...
    'SaveFormat', 'Timeseries', 'Decimation', 'max(1, round(dt*decim/ts_noise))');
add_line(mdl, 'V2V Channel/1', 'log_w/1', 'autorouting', 'on');

add_block('simulink/Sinks/To Workspace', [mdl '/log_lead'], ...
    'Position', [500 400 580 430], 'VariableName', 'lead', ...
    'SaveFormat', 'Timeseries', 'Decimation', 'decim');
add_line(mdl, 'Lead Vehicle/1', 'log_lead/1', 'autorouting', 'on');

% =======================================================================
% Solver / configuration
% =======================================================================
% Fixed-step ode4 (classical RK4): the channel is a piecewise-constant
% discrete signal of period ts_noise, so a variable-step solver would take a
% zero-crossing hit on every hold boundary.  dt = 1e-3 divides ts_noise =
% 1e-2 exactly.  RK4 at 1 ms is far inside the stability limit even for
% Table II's fastest lag (tau = 0.0257 s -> pole at 39 rad/s, h*|lambda|
% = 0.039 << 2.78).
set_param(mdl, 'Solver', 'ode4', 'FixedStep', 'dt', ...
    'StartTime', '0', 'StopTime', 'Tend', ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'off', 'SaveState', 'off', ...
    'ReturnWorkspaceOutputs', 'on');

save_system(mdl, fullfile(savePath, [mdl '.slx']));
fprintf('built %s\n', fullfile(savePath, [mdl '.slx']));
end

% =======================================================================
function buildLeadVehicle(sys)
%  a0(t) = amp*sin(omega*(t-t0)) gated to [t0, t0+dur]; integrate twice.
%  Built from a Sine Wave plus two Steps rather than a Fcn/MATLAB Function
%  block so the whole model stays inside the core block set (no interpreted
%  MATLAB in the loop -> the model is code-generatable as it stands).

add_block('simulink/Sources/Sine Wave', [sys '/Sine'], ...
    'Position', [30 30 80 80], ...
    'Amplitude', 'a0_amp', 'Frequency', 'a0_omega', ...
    'Phase', '-a0_omega*a0_t0', 'Bias', '0', 'SampleTime', '0');

add_block('simulink/Sources/Step', [sys '/Step On'], ...
    'Position', [30 110 80 160], 'Time', 'a0_t0', 'Before', '0', 'After', '1');
add_block('simulink/Sources/Step', [sys '/Step Off'], ...
    'Position', [30 180 80 230], 'Time', 'a0_t0+a0_dur', 'Before', '0', 'After', '1');
add_block('built-in/Sum', [sys '/Window'], ...
    'Position', [120 130 150 200], 'IconShape', 'rectangular', 'Inputs', '+-');

add_block('built-in/Product', [sys '/Gate'], 'Position', [190 60 220 120], 'Inputs', '2');

add_block('built-in/Integrator', [sys '/v0'], ...
    'Position', [270 60 300 90], 'InitialCondition', 'v_ss');
add_block('built-in/Integrator', [sys '/x0'], ...
    'Position', [340 60 370 90], 'InitialCondition', 'x0_lead');

add_block('built-in/Mux', [sys '/Mux'], 'Position', [420 50 425 130], 'Inputs', '3');
add_block('built-in/Outport', [sys '/leadState'], 'Position', [470 80 500 100]);

add_line(sys, 'Step On/1',  'Window/1');
add_line(sys, 'Step Off/1', 'Window/2');
add_line(sys, 'Sine/1',     'Gate/1');
add_line(sys, 'Window/1',   'Gate/2');
add_line(sys, 'Gate/1',     'v0/1');
add_line(sys, 'v0/1',       'x0/1');
add_line(sys, 'x0/1',       'Mux/1');       % [1] = x0
add_line(sys, 'v0/1',       'Mux/2', 'autorouting', 'on');   % [2] = v0
add_line(sys, 'Gate/1',     'Mux/3', 'autorouting', 'on');   % [3] = a0
add_line(sys, 'Mux/1',      'leadState/1');
end

% =======================================================================
function buildChannel(sys)
%  w = noise_bias + noise_K * z ,  z_j = 1{U_j < gamma_j},  U ~ Uniform(0,1)
%
%  The 16N Bernoulli draws come from ONE Uniform Random Number block
%  thresholded against the tiled Table-I means; noise_K then applies the
%  binary weights 2^-j AND sums each link's 16 bits in a single matrix gain,
%      noise_K = (1/rho) * kron(eye(N), [2^0 2^-1 ... 2^-15]).
%
%  All three noise modes reduce to a choice of (noise_K, noise_bias):
%      stochastic : K = (1/rho)*kron(...)   bias = (1 - 1/rho)*ones(N,1)
%      mean       : K = 0                   bias = E[w]*ones(N,1)   (eq. 13)
%      none       : K = 0                   bias = ones(N,1)
%  so the diagram is identical in every mode: no switch, no dead branch.

add_block('simulink/Sources/Uniform Random Number', [sys '/U'], ...
    'Position', [30 40 90 90], ...
    'Minimum', '0', 'Maximum', '1', 'Seed', 'seed_vec', 'SampleTime', 'ts_noise');

add_block('built-in/Constant', [sys '/TableI_gammas'], ...
    'Position', [30 120 90 160], 'Value', 'gamma_rep');

add_block('simulink/Logic and Bit Operations/Relational Operator', [sys '/lt'], ...
    'Position', [140 60 180 130], 'Operator', '<');

add_block('simulink/Signal Attributes/Data Type Conversion', [sys '/toDouble'], ...
    'Position', [220 80 260 110], 'OutDataTypeStr', 'double');

add_block('built-in/Gain', [sys '/weights'], ...
    'Position', [300 75 350 115], 'Gain', 'noise_K', 'Multiplication', 'Matrix(K*u)');

add_block('simulink/Math Operations/Bias', [sys '/offset'], ...
    'Position', [390 80 430 110], 'Bias', 'noise_bias');

add_block('built-in/Outport', [sys '/w'], 'Position', [470 85 500 105]);

add_line(sys, 'U/1',             'lt/1');
add_line(sys, 'TableI_gammas/1', 'lt/2');
add_line(sys, 'lt/1',            'toDouble/1');
add_line(sys, 'toDouble/1',      'weights/1');
add_line(sys, 'weights/1',       'offset/1');
add_line(sys, 'offset/1',        'w/1');
end

% =======================================================================
function buildPlatoon(sys)
%  Vector states x, v, a (each width N) plus the one-vehicle look-ahead shift.

add_block('built-in/Inport', [sys '/leadState'], 'Position', [20 40 50 60], 'Port', '1');
add_block('built-in/Inport', [sys '/w'],         'Position', [20 400 50 420], 'Port', '2');

add_block('built-in/Demux', [sys '/SplitLead'], ...
    'Position', [90 30 95 90], 'Outputs', '3');

% ---- integrator chain:  adot -> a -> v -> x ---------------------------
add_block('built-in/Integrator', [sys '/aInt'], ...
    'Position', [640 200 670 230], 'InitialCondition', 'a_init');
add_block('built-in/Integrator', [sys '/vInt'], ...
    'Position', [710 200 740 230], 'InitialCondition', 'v_init');
add_block('built-in/Integrator', [sys '/xInt'], ...
    'Position', [780 200 810 230], 'InitialCondition', 'x_init');

% ---- predecessor signals: pred = PredSel * [lead ; own] ---------------
names = {'x', 'v', 'a'};
for k = 1:3
    n = names{k};
    add_block('built-in/Mux', [sys '/aug_' n], ...
        'Position', [160, 60 + 90*(k-1), 165, 120 + 90*(k-1)], 'Inputs', '2');
    add_block('built-in/Gain', [sys '/pred_' n], ...
        'Position', [210, 70 + 90*(k-1), 260, 110 + 90*(k-1)], ...
        'Gain', 'PredSel', 'Multiplication', 'Matrix(K*u)');
    add_line(sys, sprintf('SplitLead/%d', k), ['aug_' n '/1'], 'autorouting', 'on');
    add_line(sys, ['aug_' n '/1'], ['pred_' n '/1']);
end
add_line(sys, 'leadState/1', 'SplitLead/1');

% ---- delta = x - x_pred + hw*v + d   (Definition 1 + eq. 2) ----------
add_block('built-in/Gain', [sys '/hw_v'], 'Position', [310 250 350 290], 'Gain', 'hw');
add_block('built-in/Constant', [sys '/d'], 'Position', [310 320 350 350], 'Value', 'd');
add_block('built-in/Sum', [sys '/Sum_delta'], ...
    'Position', [400 230 430 330], 'IconShape', 'rectangular', 'Inputs', '+-++');

add_line(sys, 'xInt/1',   'Sum_delta/1', 'autorouting', 'on');
add_line(sys, 'pred_x/1', 'Sum_delta/2', 'autorouting', 'on');
add_line(sys, 'hw_v/1',   'Sum_delta/3', 'autorouting', 'on');
add_line(sys, 'd/1',      'Sum_delta/4', 'autorouting', 'on');
add_line(sys, 'vInt/1',   'hw_v/1',  'autorouting', 'on');

% ---- feedforward:  ka * (w .* a_pred)   (eq. 6, first term) -----------
add_block('built-in/Product', [sys '/ff_mult'], ...
    'Position', [310 380 340 430], 'Inputs', '2', 'Multiplication', 'Element-wise(.*)');
add_block('built-in/Gain', [sys '/ka'], 'Position', [390 390 430 420], 'Gain', 'ka');
add_line(sys, 'w/1',       'ff_mult/1', 'autorouting', 'on');
add_line(sys, 'pred_a/1',  'ff_mult/2', 'autorouting', 'on');
add_line(sys, 'ff_mult/1', 'ka/1');

% ---- feedback: kv (v - v_pred)  and  kp delta ------------------------
add_block('built-in/Sum', [sys '/dv'], ...
    'Position', [310 460 340 510], 'IconShape', 'rectangular', 'Inputs', '+-');
add_block('built-in/Gain', [sys '/kv'], 'Position', [390 470 430 500], 'Gain', 'kv');
add_block('built-in/Gain', [sys '/kp'], 'Position', [480 270 520 300], 'Gain', 'kp');

add_line(sys, 'vInt/1',   'dv/1', 'autorouting', 'on');
add_line(sys, 'pred_v/1', 'dv/2', 'autorouting', 'on');
add_line(sys, 'dv/1',     'kv/1');
add_line(sys, 'Sum_delta/1', 'kp/1', 'autorouting', 'on');

% ---- u = ka w a_pred - kv dv - kp delta ------------------------------
add_block('built-in/Sum', [sys '/Sum_u'], ...
    'Position', [560 280 590 420], 'IconShape', 'rectangular', 'Inputs', '+--');
add_line(sys, 'ka/1', 'Sum_u/1', 'autorouting', 'on');
add_line(sys, 'kv/1', 'Sum_u/2', 'autorouting', 'on');
add_line(sys, 'kp/1', 'Sum_u/3', 'autorouting', 'on');

% ---- adot = (u - a) ./ tau   (eq. 1, second line) --------------------
add_block('built-in/Sum', [sys '/u_minus_a'], ...
    'Position', [540 160 570 210], 'IconShape', 'rectangular', 'Inputs', '+-');
add_block('built-in/Gain', [sys '/invTau'], ...
    'Position', [595 165 630 205], 'Gain', 'tau_inv', ...
    'Multiplication', 'Element-wise(K.*u)');

add_line(sys, 'Sum_u/1',     'u_minus_a/1', 'autorouting', 'on');
add_line(sys, 'aInt/1',      'u_minus_a/2', 'autorouting', 'on');
add_line(sys, 'u_minus_a/1', 'invTau/1');
add_line(sys, 'invTau/1',    'aInt/1');
add_line(sys, 'aInt/1',      'vInt/1');
add_line(sys, 'vInt/1',      'xInt/1');

% ---- outputs ----------------------------------------------------------
outs = {'x', 'v', 'a', 'delta', 'u'};
srcs = {'xInt/1', 'vInt/1', 'aInt/1', 'Sum_delta/1', 'Sum_u/1'};
for k = 1:numel(outs)
    add_block('built-in/Outport', [sys '/' outs{k}], ...
        'Position', [880, 60 + 50*(k-1), 910, 80 + 50*(k-1)], 'Port', num2str(k));
    add_line(sys, srcs{k}, [outs{k} '/1'], 'autorouting', 'on');
end

% feed the integrator outputs back into the predecessor muxes
add_line(sys, 'xInt/1', 'aug_x/2', 'autorouting', 'on');
add_line(sys, 'vInt/1', 'aug_v/2', 'autorouting', 'on');
add_line(sys, 'aInt/1', 'aug_a/2', 'autorouting', 'on');
end
