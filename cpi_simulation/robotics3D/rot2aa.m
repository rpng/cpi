function [k, theta] = rot2aa(R)

% [k, theta] = rot2aa(R)
%
% rotation matrix to axis angle represnetation
% this just gives the positive set of solution.
% Another solution [-theta, -k]
% this is ill-conditioned when theta is small

theta = acos((trace(R)-1)/2);


if theta == 0
    k = [0 ; 0 ; 0];
else
    if theta < 0.01
        warning('theta is too small: bad conditioned problem');
    end
    k = [R(3,2)-R(2,3) ; R(1,3)-R(3,1) ; R(2,1)-R(1,2)]/2/sin(theta);
end
        
    