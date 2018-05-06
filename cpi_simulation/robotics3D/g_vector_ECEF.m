function g_ECEF = g_vector_ECEF(pos_ECEF)
% returns the gravity vector at a given position expressed wrt. ECEF 
% coordinates, based on a spherical
% harmonics gravity potential up to n=2, m=0 (Values based on WGS-84)
% This will not yield apparent gravity vector (need to add -w_earth x
% w_earth x P)
% Source: A. Chatfield, "Fundamentals of High Accuracy Inertial
% Navigation", 1997.

% constants
a = 6378137.0; %m
C20bar = -4.8416685e-4;
GM = 3.986005e14; % m^3/s^2


% get geocentric longitude and latitude (Eqs. 7.36-7.38)
x = pos_ECEF(1);
y = pos_ECEF(2);
z = pos_ECEF(3);
P = norm(pos_ECEF);

sqrtx2y2 = sqrt(x^2+y^2);
if sqrtx2y2 ~= 0
    sinl = y/sqrtx2y2;
    cosl = x/sqrtx2y2;
else
    % singular case, arbitrary settings
    sinl=0;
    cosl=1;
end

sinpc = z/P;
cospc = sqrtx2y2/P;


% get components of g in geocentric north west up
% from Eq. 1.22 or also from Eq. 7.35.
g_NWU = -GM/P^2 * [3*sqrt(5)*C20bar*(a/P)^2*sinpc*cospc;
                  0;
                  1 + 3/2*sqrt(5)*C20bar*(a/P)^2*(3*sinpc^2-1)];
              

% transform to ECEF
ECEF_R_NWU = [-cosl*sinpc   -sinl*sinpc cospc;
               sinl         -cosl       0;
               cosl*cospc    sinl*cospc sinpc ]'; % adapted from eq. 2.51, note transpose

g_ECEF = ECEF_R_NWU * g_NWU;

