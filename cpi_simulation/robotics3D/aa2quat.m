function quat = aa2quat(k,theta)

quat(4,1) = cos(theta/2);

quat(1:3,1) = k*sin(theta/2);