function [Omega]=para_Omega_calculation(p)

% para_Omega_calculation(p) takes a vector and computes the para-Omega matrix
Omega = zeros(4);
Omega(1,1) = 0;
Omega(1,2) = p(3,1) ;
Omega(1,3) = -p(2,1) ;
Omega(1,4) = -p(1,1) ;

Omega(2,1) = -p(3,1) ;
Omega(2,2) = 0;
Omega(2,3) = p(1,1) ;
Omega(2,4) = -p(2,1) ;

Omega(3,1) = p(2,1) ;
Omega(3,2) = -p(1,1) ;
Omega(3,3) = 0;
Omega(3,4) = -p(3,1) ;

Omega(4,1) = p(1,1) ;
Omega(4,2) = p(2,1) ;
Omega(4,3) = p(3,1) ;
Omega(4,4) = 0 ;


