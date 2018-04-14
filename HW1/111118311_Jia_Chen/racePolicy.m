function [vel, dgamma] = racePolicy(obs)
% You must implement this function. The input/output of the function must be as follows.
% Use m_main to test your implementation. 
% Inputs:
%   obs: 1*2 vector for the values of two light sensors
% Outputs:
%   vel: the desired veolicy. You can return whatever value, 
%        but the bicycle environment will clip it to [0, 1]
%   dgamma: change of heading angle of the bike in degree. It will be clipped by [-5, 5], and the accumulated steering angle is bounded by [-85, 85]


% simple policy, keep going straight
left = obs(2);
right = obs(1);
vel = 1;
dgamma = 5;

persistent p_gamma;
if isempty(p_gamma) %first timestep
    p_gamma = dgamma;
else                %after first timestep
    p_gamma = p_gamma + dgamma;
end
%disp(p_gamma);


if left == 0 && right == 0
    vel = 1;
    dgamma = 0;
elseif left == 0 && right == 1
    vel = 1;
    dgamma = 5;
elseif right == 1 && left == 0
    vel = 1;
    dgamma = -5;
end








