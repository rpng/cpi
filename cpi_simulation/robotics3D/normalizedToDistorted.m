function [xd,yd] = normalizedToDistorted(xn,yn,k)
% using the k distortion parameters from the ASL dataset
r = sqrt(xn.^2+yn.^2);
if length(k)==5;
    radialDist = (1+k(1)*r.^2+k(2)*r.^4+k(5)*r.^6);
    tangDistX = 2*k(3)*xn.*yn + k(4)*(r.^2+2*xn.^2);
    tangDistY = k(3)*(r.^2+2*yn.^2) + 2*k(4)*xn.*yn;
    xd = radialDist.*xn + tangDistX;
    yd = radialDist.*yn + tangDistY;
else
    radialDist = (1+k(1)*r.^2+k(2)*r.^4);
    tangDistX = 2*k(3)*xn.*yn + k(4)*(r.^2+2*xn.^2);
    tangDistY = k(3)*(r.^2+2*yn.^2) + 2*k(4)*xn.*yn;
    xd = radialDist.*xn + tangDistX;
    yd = radialDist.*yn + tangDistY;
end