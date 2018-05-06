function [C] = cayley2rotbar(s)

C = ( (1-s'*s) * eye(3) + 2 * skewsymm(s) + 2 * (s * s'))';

%C2 = (eye(3) - skewsymm(s))^-1 * (eye(3) + skewsymm(s));