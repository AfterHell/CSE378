% obstacles is m*6 matrix for m triangular obstacles
% each row is the list of coordinate of the obstacle: X1 Y1 X2 Y2 X3 Y3
obstacles = [10 -20, 20, -30, 25, -10; 
             -10, 10, -20, 20, -15, 10]; 

% obstacles = [10 -20, 20, -30, 25, -10];

% obstacles = [];

robotEnv = M_TwoLinkArm(obstacles);

nStep = 40;
theta1s = pi*(0:nStep)/nStep;
theta2s = 2*pi*(0:nStep)/nStep;
isFail = robotEnv.execute(theta1s, theta2s, 1);



% % obstacles is m*6 matrix for m triangular obstacles
% % each row is the list of coordinate of the obstacle: X1 Y1 X2 Y2 X3 Y3
% obstacles = [10 -20, 20, -30, 25, -10; 
             % -10, 10, -20, 20, -15, 10]; 

% robotEnv = M_TwoLinkArm(obstacles); % a robot arm instance
% [Node, Edge] = createPlanner(robotEnv); % create RRT planner
% startXY = [20, 10]'; % the starting and end points of the tip joint
% endXY = [10, 20]';
% [theta1s, theta2s] = plan(Node, Edge, startXY, endXY);

% isFail = robotEnv.execute(theta1s, theta2s, 1);