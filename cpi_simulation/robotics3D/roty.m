function r = roty(t)
% roty: rotation about y-axi-
	ct = cos(t);
	st = sin(t);
	r =    [ct	0	st;
		0	1	0;
		-st	0	ct];
