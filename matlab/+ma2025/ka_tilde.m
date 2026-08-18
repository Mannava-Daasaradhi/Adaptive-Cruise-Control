function kat = ka_tilde(ka, rho, mode, g)
%KA_TILDE  Effective feedforward gain k_a_tilde = ka * E[w], eq. (12), and its
%          worst-case interval endpoints, eq. (20) / the set I of Sec. III-C.
%
%   kat = MA2025.KA_TILDE(ka, rho)            -> mean value, ka * E[w]   (12)
%   kat = MA2025.KA_TILDE(ka, rho, 'mean')    -> same
%   kat = MA2025.KA_TILDE(ka, rho, 'low')     -> (1 - 1/rho) * ka   (inf I)
%   kat = MA2025.KA_TILDE(ka, rho, 'high')    -> (1 + 1/rho) * ka   (sup I)
%   kat = MA2025.KA_TILDE(ka, rho, 'interval')-> [low high]
%
%   The robust analysis of Theorem 2 must hold for EVERY k_a_tilde in
%
%       I := [ (1 - 1/rho) ka , (1 + 1/rho) ka ]
%
%   because the gamma_{i,j} are not known a priori.  Condition (28a) is
%   binding at the HIGH end, condition (28b) at the LOW end.
%
%   See also MA2025.EXPECTED_W, MA2025.CHECK_GAINS.

if nargin < 3 || isempty(mode), mode = 'mean'; end
if nargin < 4, g = []; end

switch lower(mode)
    case 'mean',     kat = ka * ma2025.expected_w(rho, g);
    case 'low',      kat = (1 - 1/rho) * ka;
    case 'high',     kat = (1 + 1/rho) * ka;
    case 'interval', kat = [(1 - 1/rho) * ka, (1 + 1/rho) * ka];
    case 'none',     kat = ka;                    % noiseless channel, w == 1
    otherwise, error('ma2025:ka_tilde:mode', 'unknown mode ''%s''', mode);
end
end
