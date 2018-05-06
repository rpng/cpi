function pointNorm = pixelToNormalized(pointPixel,C)
% pointPixel = nx2 x/y pixel coordinates
% C.kd2p =  intrinsic parameters [fx fy cpx cpy]
% C.kn2d = 1x4 inverse distortion parameters
xp = pointPixel(:,1)';
yp = pointPixel(:,2)';
K = [C.kd2p(1) 0 C.kd2p(3);0 C.kd2p(2) C.kd2p(4); 0 0 1];
pixelHomVector = [xp;yp;xp*0+1];
distortedHomVector = K\pixelHomVector;
% [xd,yd] = normalizedToDistorted(distortedHomVector(1,:)',distortedHomVector(2,:)',C.kd2n);
pointNorm = comp_distortion_oulu(distortedHomVector(1:2,:),C.kn2d)';

% pointNorm = [xd,yd];

