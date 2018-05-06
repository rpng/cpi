function [C] = cayley2rot(s)

C = ( (1-s'*s) * eye(3) + 2 * skewsymm(s) + 2 * s * s')' / ( 1 + s' * s);