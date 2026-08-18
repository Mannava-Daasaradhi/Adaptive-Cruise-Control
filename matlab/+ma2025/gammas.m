function g = gammas()
%GAMMAS  Table I of Ma, Pagilla & Darbha (T-ITS 2025): gamma_{i,j} = E[z_{i,j}].
%
%   g = MA2025.GAMMAS() returns the 1x16 row vector of Bernoulli means for
%   the n = 16 binary random processes z_{i,j}, j = 0,1,...,15 that build the
%   V2V noise factor of eq. (5):
%
%       w_{i,i-1}(t) = (1 - 1/rho) + (1/rho) * sum_{j=0}^{n-1} z_{i,j}(t)/2^j
%
%   The same table is used for every link i (the paper tabulates a single
%   set of gamma_{i,j}), so the channel is statistically homogeneous.
%
%   See also MA2025.EXPECTED_W, MA2025.KA_TILDE.

g = [0.8055 0.5767 0.1829 0.2399 0.8865 0.0287 0.4899 0.1679 ...
     0.9787 0.7127 0.5005 0.4711 0.0596 0.6820 0.0424 0.0714];
end
