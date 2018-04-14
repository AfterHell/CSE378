classdef M_TwoLinkArm
% A robot environment with: a 2-link robotic arm + obstacles
% The base of the robotic arm is fixed. Each obstacle is a triangle, and there might multiple obstacles.
% This is for HW3 of Introduction to Robotics CSE 378, Spring 2018.
%
% By: Minh Hoai Nguyen (minhhoai@cs.stonybrook.edu)
% Created: 03-Apr-2018
% Last modified: 03-Apr-2018

    properties 
    % length of each arm segment L1 and L2. The list of obstacles is empty by default
        L1 = 20;
        L2 = 10;
        obstacles = [];
    end
    
    methods        
        % obstacles: m*6 matrix for m obstacles
        %   obstacles(i,:) is the coordinates of the i^th triangular obstacle: X1, Y1, X2, Y2, X3, Y3
        function obj = M_TwoLinkArm(obstacles)
            obj.obstacles = obstacles;
        end
                                
        function [jointX, jointY, tipX, tipY] = forwardKinematic(obj, theta1, theta2)
        % FORWARDKINEMATIC. Compute the location of the joints given the rotation of the joints
        % 
        % theta1: rotation of the root joint of the arm, which connects L1 and the root, in radians
        % theta2: rotation of the middle joint of the arm, which connects L1 and L2, in radians
            jointX = obj.L1*cos(theta1); %compute the location of joint 1
            jointY = obj.L1*sin(theta1);
            tipX = jointX + obj.L2*cos(theta1 + theta2); % location of joint 2
            tipY = jointY + obj.L2*sin(theta1 + theta2);
        end
                
        function isFail = execute(obj, theta1s, theta2s, shldAnimate)
        % EXECUTE. Check if the planned path is valid.
        % 
        % theta1s: a list of rotation angles that are planned for joint 1, in radians
        % theta2s: a list of rotation angles that are planned for joint 2, in radians
        % shldAnimate: a flag to turn the animation on or off, in bool
            % the length of these two list should be identical
            if isempty(theta1s) || isempty(theta2s) || length(theta1s) ~= length(theta2s)
                fprintf('Fail: Empty plan or inconsistent\n');                
                isFail = true;
                return;
            end
                                
            % Check if the joint angles between two consecutive steps is too large
            if any(abs(diff(theta1s)) >= 2*pi/36) || any(abs(diff(theta2s)) >= 2*pi/36)
                fprint('Fail: the difference between joint angles of two consecutive steps should be < 10 deg\n');
                isFail = true;            
                return;
            end
            
            % Check for collision            
            isFail = false;
            nStep = length(theta1s);
            for i=1:nStep
                theta1 = theta1s(i);
                theta2 = theta2s(i);
                % play animation
                if shldAnimate
                    clf;
                    obj.show(theta1, theta2);
                    pause(0.1);
                end
                
                % if the arm collide with any obstables, the plan fails
                if obj.checkCollision(theta1, theta2)
                    fprintf('Fail: collision occurs\n');
                    isFail = true;
                    return;
                end
            end
            fprintf('Successfully executing the plan\n');
        end
        
        function collision = checkCollision(obj, theta1, theta2)
        % COLLISION. Check if the arm collides with any obstables.
        % 
        % It first computes the 2 line segments of the arm by function *forwardKinematic*. If any of the line segments
        % of the arm intersects the line segments of the obstacle, collision happens.
        % 
        % theta1: rotation of the root joint of the arm, which connects L1 and the root, in radians
        % theta2: rotation of the middle joint of the arm, which connects L1 and L2, in radians 
            % no obstacle, no collision
            nObstacle = size(obj.obstacles, 1);
            if nObstacle == 0
                collision = false;
            else
                % compute the joint location by *forwardKinematic*
                [jointX, jointY, tipX, tipY] = obj.forwardKinematic(theta1, theta2);
                bP = [0; 0];
                jP = [jointX; jointY];
                tP = [tipX; tipY];
                % each obstacle is a triangle. Test if any of the arm segment intersect one of the three boundaries of 
                % the triangle
                for j=1:nObstacle
                    obstacle = obj.obstacles(j,:);
                    P1 = obstacle([1;2])';
                    P2 = obstacle([3;4])';
                    P3 = obstacle([5;6])';
                    % we have 2 segments for the arm and 3 segments for the obstacle, so there are 6 tests in total.
                    % if any one of the 6 test finds the intersection, then collision happens
                    if isIntersect(bP, jP, P1, P2) || isIntersect(bP, jP, P1, P3) || ...
                       isIntersect(bP, jP, P2, P3) || isIntersect(tP, jP, P1, P2) || ...
                       isIntersect(tP, jP, P1, P3) || isIntersect(tP, jP, P2, P3)
                   
                        collision = true;
						break;
                    else
                        collision = false;
                    end
                end
            end
        end
        
                
        function animate(obj, theta1s, theta2s)
        % ANIMATE. It will play the animation without collision checking
        % 
        % theta1s: a list of rotation angles that are planned for joint 1, in radians
        % theta2s: a list of rotation angles that are planned for joint 2, in radians
            for i=1:length(theta1s)                
                theta1 = theta1s(i);
                theta2 = theta2s(i);
                clf;
                obj.show(theta1, theta2);
                pause(0.1);                
            end            
        end
        
        function show(obj, theta1, theta2)
        % SHOW. Plot the current state of the environment.
        % 
        % It plots the locations of the different parts of the arm, including L1, L2, root joint and middle joint.
        % It also plots the obstacles.
            baseX = 0;
            baseY = 0;
            % compute the joint positions
            [jointX, jointY, tipX, tipY] = obj.forwardKinematic(theta1, theta2);
                        
            % plot all the obstacles
            for j=1:size(obj.obstacles, 1)
                obstacle = obj.obstacles(j,:);
                fill(obstacle([1,3,5]), obstacle([2,4,6]), [0.5, 0.5, 0.5]);
                hold on;
            end
            
            % plot L1 as a blue line segment
            line([baseX, jointX], [baseY, jointY], 'LineWidth', 10, 'Color', 'b');
            % plot L2 as a cyan line segment
            line([jointX, tipX], [jointY, tipY], 'LineWidth', 10, 'Color', 'c');
            hold on;
            % plot the joints
            scatter([baseX, jointX, tipX], [baseY, jointY, tipY], 120, 'm', 'filled');
            axis equal;
            axis([-40, 40, -40, 40]);            
        end
    end
end


