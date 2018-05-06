function [q]=quat_inv(p)

% quaternion_inv(p) takes a unit quaternion p and returns its inverse

q(1,1) = -p(1,1);
q(2,1) = -p(2,1);
q(3,1) = -p(3,1);
q(4,1) = p(4,1);