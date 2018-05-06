function [S]=neg_cross_product(p)

% neg_cross_product(p) takes a vector p and returns the negative skew-symmetric crossproduct matrix
S = zeros(3);
S(1,1) = 0;
S(1,2) = p(3,1);
S(1,3) = -p(2,1) ;

S(2,1) = -p(3,1) ;
S(2,2) = 0;
S(2,3) = p(1,1) ;

S(3,1) = p(2,1) ;
S(3,2) = -p(1,1) ;
S(3,3) = 0;
