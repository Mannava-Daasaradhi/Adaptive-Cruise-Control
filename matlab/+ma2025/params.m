function P = params(caseName, varargin)
%PARAMS  Parameter sets for the five numerical cases of Ma et al. (2025) Sec. IV.
%
%   P = MA2025.PARAMS('case1')            % string stable, hw = 0.95 s
%   P = MA2025.PARAMS('case2')            % string UNstable, hw = 0.65 s
%   P = MA2025.PARAMS('case3')            % optimal ka*, hw = h*_lb = 0.88 s
%   P = MA2025.PARAMS('case4a'|'case4b')  % platoon-length comparison
%   P = MA2025.PARAMS('case5')            % heterogeneous parasitic lags
%   P = MA2025.PARAMS('case1', 'noise_mode', 'mean', 'N', 6, ...)
%
%   Common numerical values, Sec. IV-A of the paper:
%       N     = 12 followers          tau0 = 0.5 s        d = 5 m
%       rho   = 5  (SNR 13.98 dB)     v_ss = 25 m/s (90 km/h)
%       n     = 16 communication channels, gamma_{i,j} from Table I
%       a0(t) = 0.5 sin(0.1 (t - 10)) for 10 < t < 10 + 20 pi s, else 0
%
%   NOISE_MODE selects how w_{i,i-1}(t) is realised in the Simulink model:
%       'stochastic' (default) -- 16 Bernoulli bits redrawn every ts_noise,
%                                 i.e. eq. (5); this is what produced Fig. 8
%       'mean'                 -- w == E[w] of eq. (12); the equivalent
%                                 DETERMINISTIC system (13) of Remark 5
%       'none'                 -- w == 1, ideal channel (Remark 3 limit)
%
%   Fields consumed by the Simulink model are collected in P (all scalars or
%   N-vectors) and pushed to the model workspace by MA2025.RUN_CASE.
%
%   See also MA2025.RUN_CASE, MA2025.LEAD_ACCEL, BUILD_CACC_MODEL.

if nargin < 1, caseName = "case1"; end
caseName = string(caseName);

% ---- values shared by every case (Sec. IV-A) ---------------------------
P.name      = caseName;
P.N         = 12;            % number of FOLLOWING vehicles
P.tau0      = 0.5;           % upper bound on the parasitic actuation lag [s]
P.d         = 5;             % standstill / minimum spacing [m]
P.rho       = 5;             % SNR factor, rho = min|s|/|n| (Definition 3)
P.v_ss      = 25;            % steady-state platoon speed [m/s] (90 km/h)
P.nbits     = 16;            % n, number of communication channels
P.gammas    = ma2025.gammas();
P.a0_amp    = 0.5;           % [m/s^2]
P.a0_omega  = 0.1;           % [rad/s]
P.a0_t0     = 10;            % maneuver start [s]
P.a0_dur    = 20*pi;         % maneuver duration [s] -> exactly one period
P.Tend      = 200;           % simulation horizon [s]
P.dt        = 1e-3;          % fixed solver step [s]
P.ts_noise  = 1e-2;          % zero-order hold on w(t) [s]
P.decim     = 10;            % logging decimation -> 100 Hz records
P.seed      = 1;             % base RNG seed for the channel
P.noise_mode = "stochastic";

% ---- per-case gains ----------------------------------------------------
switch lower(string(caseName))
    case "case1"                                   % Figs. 5,6,7,8
        P.ka = 0.5;    P.hw = 0.95;  P.kv = 0.63;  P.kp = 0.009;
        P.desc = "Case 1: hw = 0.95 s > h_lb = 0.9375 s -- robustly string stable";
    case "case2"                                   % Figs. 9,10
        P.ka = 0.5;    P.hw = 0.65;  P.kv = 0.63;  P.kp = 0.009;
        P.desc = "Case 2: hw = 0.65 s < h_lb = 0.9375 s -- string UNstable";
    case "case3"                                   % Figs. 11,12
        P.ka = ma2025.ka_star(5);  P.hw = 0.88;  P.kv = 0.85;  P.kp = 0.003;
        P.desc = "Case 3: ka = ka* = 0.3183, hw = 0.88 s ~ h*_lb = 0.8727 s";
    case "case4a"                                  % Fig. 13, long headway
        P.ka = 0.5;    P.hw = 0.95;  P.kv = 0.63;  P.kp = 0.009;
        P.desc = "Case 4a: hw = 0.95 s (Case 1 design) -- platoon length";
    case "case4b"                                  % Fig. 13, short headway
        P.ka = ma2025.ka_star(5);  P.hw = 0.88;  P.kv = 0.85;  P.kp = 0.003;
        P.desc = "Case 4b: hw = 0.88 s (Case 3 design) -- platoon length";
    case "case5"                                   % Figs. 14,15
        P.ka = 0.5;    P.hw = 0.95;  P.kv = 0.63;  P.kp = 0.009;
        P.desc = "Case 5: Case 1 design, heterogeneous tau_i (Table II)";
    otherwise
        error('ma2025:params:unknownCase', ...
              'unknown case "%s" (case1..case3, case4a, case4b, case5)', caseName);
end

% ---- name/value overrides ---------------------------------------------
for k = 1:2:numel(varargin)
    P.(char(varargin{k})) = varargin{k+1};
end

% ---- derived quantities (after overrides so N/hw changes propagate) ----
if lower(string(P.name)) == "case5"
    t = ma2025.hetero_taus();
    P.tau_vec = t(1:min(P.N, numel(t)));
    if P.N > numel(t)                              % extend by cycling
        P.tau_vec = t(mod(0:P.N-1, numel(t)) + 1);
    end
else
    P.tau_vec = P.tau0 * ones(P.N, 1);
end

P.gamma      = P.kv + P.hw * P.kp;                 % gamma := kv + hw kp
P.Ew         = ma2025.expected_w(P.rho, P.gammas);
P.ka_tilde   = P.ka * P.Ew;                        % eq. (12)
P.ka_int     = ma2025.ka_tilde(P.ka, P.rho, 'interval');
P.h_lb       = ma2025.h_lb(P.ka, P.rho, P.tau0);   % eq. (17)
P.ka_star    = ma2025.ka_star(P.rho);              % eq. (18)
P.h_star     = ma2025.h_star(P.rho, P.tau0);       % eq. (19)

% equilibrium initial condition: delta_i(0) = 0 for every follower, i.e.
% x_{i-1}(0) - x_i(0) = d + hw * v_ss  (front-to-front, point masses)
P.spacing0 = P.d + P.hw * P.v_ss;
P.x0_lead  = 0;
P.x_init   = P.x0_lead - (1:P.N)' * P.spacing0;
P.v_init   = P.v_ss * ones(P.N, 1);
P.a_init   = zeros(P.N, 1);
end
