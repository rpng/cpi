function X_ECEF = latlonalt2ECEF(lat,lon,height)

% latitude, logitude, and GPS (WGS84) altitude to ECEF
% inputs MUST be in rad

phi = lat;
lambda = lon;

% WGS84 parameters
% ellipse axes
a = 6378137.0;
b = 6356752.3142;
% e squared
e2 = 0.00669437999013;


% compute coordinates:
N = a/sqrt(1-e2*sin(phi)^2);

X = (N+height)*cos(phi)*cos(lambda);
Y = (N+height)*cos(phi)*sin(lambda);
Z = (N*(1-e2)+height)*sin(phi);


X_ECEF = [X;Y;Z]; 