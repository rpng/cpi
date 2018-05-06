%ROTVEC Rotation about arbitrary axis
%
% 	TR = AA2ROT(V, THETA)
%
% Returns a homogeneous transformation representing a rotation of THETA 
% about the vector V.
%
% See also: ROTX, ROTY, ROTZ.

% Copyright (C) 1993-2002, by Peter I. Corke
%
% CHANGES:
% 7/01	unitize the vector
% $Log: aa2rot.m,v $
% Revision 1.1.1.1  2007/06/28 03:45:21  faraz
% this is a first try to set bundle adjustment on CVS (version 6) 
%
% Revision 1.3  2002/04/01 11:47:16  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 145 $

function r = aa2rot(v, t)

	v = v/norm(v);
	ct = cos(t);
	st = sin(t);
	vt = 1-ct;
	v = v(:);
	r =    [ct		-v(3)*st	v(2)*st
		v(3)*st		ct		-v(1)*st
		-v(2)*st	v(1)*st		ct	];
	r = v*v'*vt+r;
    