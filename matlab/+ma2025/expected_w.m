function Ew = expected_w(rho, g)
%EXPECTED_W  E[w_{i,i-1}] for the n-bit multiplicative V2V channel, eq. (5).
%
%   Ew = MA2025.EXPECTED_W(rho)     uses Table I (MA2025.GAMMAS).
%   Ew = MA2025.EXPECTED_W(rho, g)  uses the supplied Bernoulli means g.
%
%   Taking expectations term-by-term in eq. (5) and using E[z_{i,j}] = gamma_j,
%
%       E[w] = (1 - 1/rho) + (1/rho) * sum_{j=0}^{n-1} gamma_j / 2^j        (12)
%
%   Because sum_j gamma_j/2^j lies in (0, 2), E[w] lies in (1-1/rho, 1+1/rho),
%   i.e. the mean channel gain need NOT be unity -- with Table I and rho = 5,
%   E[w] = 1.0482, so the channel is biased slightly optimistic.
%
%   See also MA2025.KA_TILDE, MA2025.GAMMAS.

if nargin < 2 || isempty(g)
    g = ma2025.gammas();
end
n  = numel(g);
w  = 2.^-(0:n-1);           % 1, 1/2, 1/4, ...
Ew = (1 - 1/rho) + (1/rho) * (g(:).' * w(:));
end
