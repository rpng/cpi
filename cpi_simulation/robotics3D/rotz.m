function r = rotz(t)
% rotz: rotation about z-axis

ct = cos(t);
st = sin(t);
r = [ct	-st	0
     st	ct	0
     0	0	1];
