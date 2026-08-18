function R = run_case(P, mdl)
%RUN_CASE  Simulate one parameter set on the Simulink model, return tidy data.
%
%   R = MA2025.RUN_CASE(P)            % P from MA2025.PARAMS
%   R = MA2025.RUN_CASE(P, 'cacc_platoon')
%
%   Everything the model needs is pushed through a Simulink.SimulationInput
%   rather than the base workspace, so runs are side-effect free and can be
%   parallelised or replayed without hidden state.
%
%   Returned struct R:
%       t        (M x 1)    time [s], decimated to 1/(dt*decim) Hz
%       x,v,a    (M x N)    follower position [m], speed [m/s], accel [m/s^2]
%       u        (M x N)    commanded acceleration [m/s^2]
%       delta    (M x N)    CTHP spacing error, eq. (2)   -- the paper's plots
%       e        (M x N)    CSP spacing error  = delta - hw*v  (Definition 1)
%       gap      (M x N)    physical front-to-front spacing x_{i-1} - x_i [m]
%       w        (M x N)    realised channel factor per link
%       x0,v0,a0 (M x 1)    lead-vehicle states
%       len      (M x 1)    platoon length x_0 - x_N  [m]  (Fig. 13)
%       maxAbsDelta (1 x N) max_t |delta_i|            (Figs. 7,10,12,14)
%       maxAbsAccel (1 x N) max_t |a_i|                (Fig. 15)
%       P        the parameter struct actually used
%
%   See also MA2025.PARAMS, BUILD_CACC_MODEL.

arguments
    P   (1,1) struct
    mdl (1,:) char = 'cacc_platoon'
end

N = P.N;
nb = P.nbits;

% ---- channel realisation matrices (see buildChannel) -------------------
weights = 2.^-(0:nb-1);                       % [1 1/2 1/4 ... 2^-15]
switch lower(string(P.noise_mode))
    case "stochastic"
        noise_K    = (1/P.rho) * kron(eye(N), weights);
        noise_bias = (1 - 1/P.rho) * ones(N,1);
    case "mean"                                % eq. (13): E[w] is constant
        noise_K    = zeros(N, nb*N);
        noise_bias = ma2025.expected_w(P.rho, P.gammas) * ones(N,1);
    case "none"                                % ideal channel, w == 1
        noise_K    = zeros(N, nb*N);
        noise_bias = ones(N,1);
    otherwise
        error('ma2025:run_case:noiseMode', ...
              'noise_mode must be stochastic | mean | none');
end

% one independent stream per (link, bit); deterministic in P.seed
seed_vec  = double(P.seed) + (0:nb*N-1)';
gamma_rep = repmat(P.gammas(:), N, 1);         % link-major, matches kron order

% ---- make sure the model exists ---------------------------------------
here = fileparts(fileparts(mfilename('fullpath')));   % matlab/
modelDir = fullfile(here, 'models');
addpath(modelDir);
if isempty(which([mdl '.slx']))
    build_cacc_model(mdl, modelDir);
end

si = Simulink.SimulationInput(mdl);
vars = struct( ...
    'N', N, 'ka', P.ka, 'kv', P.kv, 'kp', P.kp, 'hw', P.hw, 'd', P.d, ...
    'v_ss', P.v_ss, 'x0_lead', P.x0_lead, ...
    'tau_inv', 1 ./ P.tau_vec(:), ...
    'PredSel', [eye(N), zeros(N,1)], ...
    'noise_K', noise_K, 'noise_bias', noise_bias, ...
    'gamma_rep', gamma_rep, 'seed_vec', seed_vec, 'ts_noise', P.ts_noise, ...
    'x_init', P.x_init(:), 'v_init', P.v_init(:), 'a_init', P.a_init(:), ...
    'a0_amp', P.a0_amp, 'a0_omega', P.a0_omega, 'a0_t0', P.a0_t0, ...
    'a0_dur', P.a0_dur, 'dt', P.dt, 'Tend', P.Tend, 'decim', P.decim);

fn = fieldnames(vars);
for k = 1:numel(fn)
    si = si.setVariable(fn{k}, vars.(fn{k}));
end

out = sim(si);

% ---- unpack ------------------------------------------------------------
R.P     = P;
R.t     = out.x.Time;
R.x     = out.x.Data;
R.v     = out.v.Data;
R.a     = out.a.Data;
R.u     = out.u.Data;
R.delta = out.delta.Data;
R.w     = out.w.Data;

lead = out.lead.Data;
R.x0 = lead(:,1);  R.v0 = lead(:,2);  R.a0 = lead(:,3);

R.e   = R.delta - P.hw * R.v;                    % Definition 1
R.gap = [R.x0, R.x(:,1:end-1)] - R.x;            % physical spacing
R.len = R.x0 - R.x(:,end);                       % platoon length (Fig. 13)

R.maxAbsDelta = max(abs(R.delta), [], 1);
R.maxAbsAccel = max(abs(R.a),     [], 1);

% per-hop L2 amplification, ||delta_i||_2 / ||delta_{i-1}||_2
nrm = sqrt(sum(R.delta.^2, 1) * (R.t(2) - R.t(1)));
R.l2 = nrm;
R.l2ratio = nrm(2:end) ./ nrm(1:end-1);
end
