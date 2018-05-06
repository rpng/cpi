
function kd2n = computeInverseDistortion(kn2d)
[x,y] = meshgrid(-10:0.1:10,-10:0.1:10);
z = 10;
xn = x/z;
yn = y/z;
% plot(xn,yn,'.')
[xd,yd] = normalizedToDistorted(xn,yn,kn2d);
plot(xd,yd,'.')
% compute inverse distortion vector
xd = reshape(xd,[],1);
yd = reshape(yd,[],1);
xn = reshape(xn,[],1);
yn = reshape(yn,[],1);
r = sqrt(xd.^2+yd.^2);
%       k1          k2         k3           k4    
Hx = [ r.^2.*xd   r.^4.*xd   2*xd.*yd   (r.^2+2*xd.^2) r.^6.*xd];
Hy = [ r.^2.*yd   r.^4.*yd   (r.^2+2*yd.^2) 2*xd.*yd r.^6.*yd];
yx = xn-xd;
yy = yn-yd;
H = [Hx;Hy];y = [yx;yy];
kd2n = (H'*H)\(H'*y);


[xn2,yn2] = normalizedToDistorted(xd,yd,kd2n);