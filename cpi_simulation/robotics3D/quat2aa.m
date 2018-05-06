function [k, theta] = quat2aa(quat)

theta = 2*acos(quat(4,1));

if theta == 0
    k = [0 ; 0 ; 0];
    return;
end

if theta < 0.0001
    warning('theta is too small: bad conditioned problem');
end

k = quat(1:3,1)/sin(theta/2);
k = k/norm(k);