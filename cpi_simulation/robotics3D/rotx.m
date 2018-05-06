%ROTX Rotation about X axis

function r = rotx(t)
%rotx: rotation around the x-axis

	ct = cos(t);
	st = sin(t);
	r =    [1	0	0;
		    0	ct	-st;
		    0	st	ct];
