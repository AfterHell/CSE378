function Prot = rotateAxis(v, theta, P)
% You must implement this function. 
% The input/output of the function must be exactly as follows.
% Inputs:
%   v: the axis of rotation, a 3x1 column vector
%   theta: the angle of robation in radian.
%   P: 3*1 vector for a point in 3D space
% Outputs:
%   Prot: a 3*1 vector for the point P after rotation
Prot = P*cos(theta) + cross(v, P)*sin(theta) + v*(v.'*P)*(1-cos(theta));
%disp(Prot);



