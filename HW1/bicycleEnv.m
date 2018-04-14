classdef bicycleEnv < handle
% ==================> DO NOT modify this file  <===================
% Created by: Ke Ma
% Modified by: Minh Hoai Nguyen (minhhoai@cs.stonybrook.edu)
% Created: 01-Feb-2018
% Last modified: 01-Feb-2018


    properties (Access = private)
        x = 0; % current position of the bike
        y = 0;
        theta = 0; % current heading angle        
        map; % current map
        xs; ys; % finish point     
        gamma = 0; % current steering angle  
    end
        
    methods
        function obj = bicycleEnv()
            obj.reset();
        end
        
        function [traj, done] = rideBike(obj, bikePolicy, shldDisp)            
            [obs, ~] = obj.step(0, 0);
            maxStep = 5000;
            traj = zeros(2, maxStep);  
            for step=1:maxStep
                [vel, dgamma] = bikePolicy(obs); 
                [obs, done] = obj.step(vel, dgamma);
                traj(:, step) = [obj.x, obj.y];
                if done
                    break;
                end
            end
            traj = traj(:, 1:step);
            if exist('shldDisp', 'var') && shldDisp
                clf; imshow(obj.map); hold on; scatter(traj(1,:), traj(2,:), '.r');
                scatter(obj.xs, obj.ys, 'd', 'MarkerFaceColor',[0 .7 .7]); 
                if done                    
                    title(sprintf('Reach destination in %d steps', step));
                else
                    title('Fail to reach destiniation');
                end
            end
        end
        
        % reset the map and put the robot at the initial point and orientation
        function reset(obj)
            fh = load('maps.mat');
            maps = fh.maps;
            id = randi(length(maps));
            t = maps{id};
            obj.map = t.mp;
            obj.x = t.sp(1); % start position
            obj.y = t.sp(2); 
            obj.theta = -t.sp(3); % start angle
            obj.xs = t.ep(1);
            obj.ys = t.ep(2);
        end
        
    end
    
    methods (Access = private)
        % update the robot position and angle given velocity and angle
        function [obs, done] = step(obj, vel, dgamma)
            dgamma = min(max(dgamma, -5), 5);
            obj.gamma = obj.gamma + dgamma;
            [obj.x, obj.y, obj.theta] = obj.update_bicycle(obj.x, obj.y, obj.theta, vel, obj.gamma);
            % test if the sensor are on the line or not (we need three sensors)
            obs = obj.update_sensor(obj.x, obj.y, obj.theta, obj.map);
            done = obj.check_end(obj.x, obj.y, obj.xs, obj.ys);
        end
    end
    
    methods (Static, Access = private)        
        function [x, y, theta] = update_bicycle(x, y, theta, v, gamma)
            % velocity must be between 0 and 10
            v = min(v, 1);
            v = max(v, 0);
            
            % suppose the heading angle can only between -85 and 85 degree
            gamma = min(gamma, 85);
            gamma = max(gamma, -85);
            
            L = 1;
            dx = v * cosd(theta);
            dy = v * sind(theta);
            dtheta = v / L * tand(gamma);

            x = x + dx;
            y = y + dy;
            theta = theta + dtheta;
        end
        
        function s = update_sensor(x, y, theta, map)
            % constrain the position inside the playground
            mgn = 8; % margin of the playground
            [mh, mw] = size(map);
            x = min(max(mgn, round(x)), mw - mgn);
            y = min(max(mgn, round(y)), mh - mgn);
            % asix length
            ax = 4;
            p1 = [0, ax]';
            p2 = [0, -ax]';
            rmat = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];
            p1 = round(rmat * p1) + [x, y]';
            p2 = round(rmat * p2) + [x, y]';
            s1 = map(p1(2), p1(1));
            s3 = map(p2(2), p2(1));
            s = [s1, s3];
            %disp(s);
        end 
        
        % Check if the destination is within reach
        function done = check_end(x, y, xs, ys)
            d = (x - xs)^2 + (y - ys)^2;
            if d < 10
                done = 1;
            else
                done = 0;
            end
        end
    end    
end
