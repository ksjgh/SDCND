% Erion Plaku
% Introduction to Robotics
% Department of Electrical Engineering and Computer Science
% Catholic University of America
%
% http://faculty.cua.edu/plaku/
%
% Copyright 2012 by Erion Plaku
% The content may be reproduced under the obligation to acknowledge the source

classdef ManipSimulator
% Simulator class
% Methods provide access to all the functionality that you will need from
% the simulator
% Simulator is written as a class to make it even more similar with the C++
% interface
    
    methods      
        function [x, y] = GetGoalCenter(obj)
            x = obj.m_circles(1);	
            y = obj.m_circles(2);
        end
         
        function n = GetNrObstacles(obj)
            n = length(obj.m_circles) / 3 - 1;
        end
        
       function [xmin, ymin] = ClosestPointOnObstacle(obj, i, x, y)
       % [xmin, ymin] is the closest point on the i-th circle obstacle to point [x, y]
            cx   = obj.m_circles(1 + 3 * i);
            cy   = obj.m_circles(2 + 3 * i);
            r    = obj.m_circles(3 + 3 * i);
            d    = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));
            xmin = cx + r * (x - cx) / d;
            ymin = cy + r * (y - cy) / d;
        end
        
        function n = GetNrLinks(obj)
            n = length(obj.m_lengths);
        end
        
        function [x, y] = GetLinkStart(obj, i)
        % Position where the i-th link starts 
            x = obj.m_positionsX(i);
            y = obj.m_positionsY(i);
        end
        
        function [x, y] = GetLinkEnd(obj, i)
        % Position where the i-th link ends
            x = obj.m_positionsX(i + 1);
            y = obj.m_positionsY(i + 1);
        end
   
        function status = HasRobotReachedGoal(obj)
        % Robot reaches goal iff end effector touches the goal circle
            [ex, ey] = obj.GetLinkEnd(obj.GetNrLinks());
            [gx, gy] = obj.GetGoalCenter();
            status   = norm([ex, ey] - [gx, gy]) < obj.GetGoalRadius();
        end
        
   
   % You do not need any of the functions below this line to implement 
   % your potential field planner. These functions support the interface.
  
        function r = GetGoalRadius(obj)
            r = obj.m_circles(3);
        end
        
        function theta = GetLinkTheta(obj, i)
           theta = obj.m_thetas(i);
        end
        
        function a = GetLinkLength(obj, i)
            a = obj.m_lengths(i);
        end
        
        function obj = AddDeltaThetas(obj, dthetas)
            obj.m_thetas = obj.m_thetas + dthetas;
        end
   
        function obj = FK(obj)
        % Perform forward kinematics
          Mall = [1 0 0; 0 1 0; 0 0 1];
          M    = [1 0 0; 0 1 0; 0 0 1];
          n    = obj.GetNrLinks();
      
         for i = 1 : 1 : n
           ctheta = cos(obj.m_thetas(i));
           stheta = sin(obj.m_thetas(i));

           M(1, 1) = ctheta;  M(1, 2) = -stheta; M(1, 3) = obj.GetLinkLength(i) * ctheta;
	       M(2, 1) = stheta;  M(2, 2) =  ctheta; M(2, 3) = obj.GetLinkLength(i) * stheta;
        
           Mall = Mall * M;
       
           obj.m_positionsX(i + 1) = Mall(1, 3);
           obj.m_positionsY(i + 1) = Mall(2, 3);
         end
        end
        
        function [] = Draw(obj)
        % Draw manipulator, goal, and obstacles
                plot(obj.m_positionsX(1:length(obj.m_positionsX) - 1), ...
                    obj.m_positionsY(1:length(obj.m_positionsY) - 1), 'ro', 'MarkerSize', 5);
               line(obj.m_positionsX, obj.m_positionsY, 'Color', [1 0 0], 'LineWidth', 2);
              
              fill(obj.m_circles(3) * obj.m_xptsStandardCircle + obj.m_circles(1), ...
                   obj.m_circles(3) * obj.m_yptsStandardCircle + obj.m_circles(2), [0 1 0]);
              
               n = length(obj.m_circles);
               for i = 4 : 3 : n
                   fill(obj.m_circles(i + 2) * obj.m_xptsStandardCircle + obj.m_circles(i), ...
                        obj.m_circles(i + 2) * obj.m_yptsStandardCircle + obj.m_circles(i + 1), [0 0 1]);
               end
        end
    
        function a = GetSumLinkLengths(obj)
            a = sum(obj.m_lengths);
        end
                
        function obj = AddCircle(obj, x, y, r)
           n = length(obj.m_circles);
           obj.m_circles(n + 1) = x;
           obj.m_circles(n + 2) = y;
           obj.m_circles(n + 3) = r;          
        end
   
        function obj = ManipSimulator(nrLinks, length)
        % Constructor: 
        %   nrLinks: number of links
        %   length : length of each link
        
            obj.m_thetas     = zeros(1, nrLinks);
            obj.m_lengths    = length * ones(1, nrLinks);
            obj.m_positionsX = zeros(1, nrLinks + 1);
            obj.m_positionsY = zeros(1, nrLinks + 1);
            obj.m_circles    = [5 5 1];            
            obj = obj.FK();

            dthetas                  = 0 : 2 * pi / 50 : 2 * pi;
            obj.m_xptsStandardCircle = cos(dthetas);
            obj.m_yptsStandardCircle = sin(dthetas);

        end
    
    end    
        
    properties(GetAccess=public, SetAccess=private)
        m_lengths;
        m_thetas;
        m_positionsX;
        m_positionsY;
        m_xptsStandardCircle;
        m_yptsStandardCircle;
    end
    
    properties(GetAccess=public, SetAccess=public)
        m_circles;
    end
    
end

