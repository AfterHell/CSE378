function Prot = rotateQuat(q, P)
% You must implement this function. 
% The input/output of the function must be exactly as follows.
% Inputs:
%   q: 4*1 vector representing a unit quaternion
%   P: 3*1 vector for a point in 3D space
% Outputs:
%   Prot: a 3*1 vector for the point P after rotation
P1 = QuaternionMultiply(q, [0; P]);
P2 = QuaternionMultiply(P1, InverseOfQuaternion(q));
Prot = [P2(2); P2(3); P2(4)];

function result = QuaternionMultiply(q1, q2)
w1 = q1(1);
w2 = q2(1);
v1 = [q1(2);q1(3);q1(4)];
v2 = [q2(2);q2(3);q2(4)];
var1 = w1*w2 - dot(v1,v2);
var2 = w1*v2 + w2*v1 + cross(v1,v2);
result = [var1; var2];
%disp(cross(v1,v2));

function result = InverseOfQuaternion(q)
result = [q(1); -q(2); -q(3); -q(4)]/1^2;
%disp(result);

