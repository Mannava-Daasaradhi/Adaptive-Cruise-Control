function tau = hetero_taus()
%HETERO_TAUS  Table II of Ma et al. (2025): heterogeneous parasitic lags.
%
%   tau = MA2025.HETERO_TAUS() returns the 12x1 column vector of tau_i,
%   i = 1..12, drawn randomly by the authors from (0, tau0] with
%   tau0 = 0.5 s.  Used by Case 5 (their Figs. 14 and 15) to show that the
%   design certified for the homogeneous upper bound tau0 also works for any
%   realisation of the individual lags -- the "tau uncertain in (0, tau0]"
%   claim of Theorem 2.
%
%   See also MA2025.PARAMS.

tau = [0.2530; 0.1111; 0.1535; 0.3987; 0.1253; 0.0562; ...
       0.2240; 0.4101; 0.1998; 0.2375; 0.0257; 0.1761];
end
