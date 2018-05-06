function  pointPixel = normalizedToPixel(pointNorm,C)
% pointPixel = nx2 x/y pixel coordinates
% C.kd2p =  intrinsic parameters [fx fy cpx cpy]
% C.kd2n = 1x4 inverse distortion parameters
% C.kn2d = 1x4 distortion parameters
% reverse of pixelToNormalized
[xd,yd] = normalizedToDistorted(pointNorm(:,1),pointNorm(:,2),C.kn2d);
distortedHomVector = [xd,yd,xd*0+1].';
K = [C.kd2p(1) 0 C.kd2p(3);0 C.kd2p(2) C.kd2p(4); 0 0 1];
pixelHomVector = K*distortedHomVector;
pointPixel = pixelHomVector(1:2,:).';