classdef RigidBodySimulator
% Simulator class
% Methods provide access to all the functionality that you will need from
% the simulator.
% Simulator is written as a class to make it even more similar with the C++
% interface
%
% You do not need to make changes to this file
    
    methods
        function [x, y] = GetGoalCenter(obj)
            x = obj.m_circles(1);	
            y = obj.m_circles(2);
        end
         
        function r = GetGoalRadius(obj)         
            r = obj.m_circles(3);
        end

        function currVertices = GetRobotCurrentVertices(obj)
        % Vertices are stored consecutively in the vector
        % So the x-coord of the i-th vertex is at index 2 * i - 1
        % and the y-coord of the i-th vertex is at index 2 * i
            currVertices = obj.m_robotCurrVertices;
        end
        
        function [x, y, theta] = GetRobotCurrentConfig(obj)        
            x = obj.m_robotCurrConfig(1);
            y = obj.m_robotCurrConfig(2);
            theta = obj.m_robotCurrConfig(3);
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
        
        
        function status = HasRobotReachedGoal(obj)
            [gx, gy] = obj.GetGoalCenter();            
            status = norm([gx, gy] - ...
                         [obj.m_robotCurrConfig(1), obj.m_robotCurrConfig(2)]) <= obj.GetGoalRadius();
        end
       
        
       % Functions below this line are not needed for the implementation of
       % your planner. These functions are needed only by the graphical
       % interface.
 
       function obj = AddToRobotConfig(obj, dx, dy, dtheta)
            obj.m_robotCurrConfig = obj.m_robotCurrConfig + [dx, dy, dtheta];
            ctheta = cos(obj.m_robotCurrConfig(3));
            stheta = sin(obj.m_robotCurrConfig(3));
            nv2    = length(obj.m_robotCurrVertices);
            
            for i = 1 : 2 : nv2
                obj.m_robotCurrVertices(i) = ...
                    ctheta * obj.m_robotInitVertices(i) -...
                    stheta * obj.m_robotInitVertices(i + 1) + ...
                    obj.m_robotCurrConfig(1);
               obj.m_robotCurrVertices(i+1) = ...
                    stheta * obj.m_robotInitVertices(i) +...
                    ctheta * obj.m_robotInitVertices(i + 1) + ...
                    obj.m_robotCurrConfig(2);                    
            end        
        end
       
        function [] = Draw(obj)
          clf; hold on; grid on;
          set(gca, 'xlim', [-22.5 22.5]); 
          set(gca, 'ylim', [-18.5 18.5]);
  
        
         fill(obj.m_circles(3) * obj.m_xptsStandardCircle + obj.m_circles(1), ...
              obj.m_circles(3) * obj.m_yptsStandardCircle + obj.m_circles(2), [0 1 0]);
              
         n = length(obj.m_circles);
         for i = 4 : 3 : n
           fill(obj.m_circles(i + 2) * obj.m_xptsStandardCircle + obj.m_circles(i), ...
                obj.m_circles(i + 2) * obj.m_yptsStandardCircle + obj.m_circles(i + 1), [0 0 1]);
         end

          n = length(obj.m_robotCurrVertices);
          fill(obj.m_robotCurrVertices(1 : 2 : n), obj.m_robotCurrVertices(2 : 2 : n), [1 0 0]);
    
          drawnow;
        end
               
        function obj = AddCircle(obj, x, y, r)
           n = length(obj.m_circles);
           obj.m_circles(n + 1) = x;
           obj.m_circles(n + 2) = y;
           obj.m_circles(n + 3) = r;          
        end
        
        function poly = ReadPolygon(obj, in)       
            nv   = fscanf(in, '%d', [1 1]);
            poly = zeros(1, 2 * nv);
            for j = 1 : 1 : 2 * nv
                poly(j) = fscanf(in, '%f', [1 1]);
            end    
        end
    
        function obj = ReadRobot(obj, fname)
            in = fopen(fname, 'r');
            obj.m_robotInitVertices = obj.ReadPolygon(in);
            obj.m_robotCurrVertices = obj.m_robotInitVertices;
            obj.m_robotCurrConfig   = [0 0 0];   
            fclose(in);
        end
        
        function obj = RigidBodySimulator(fnameRobot)        
            obj = obj.ReadRobot(fnameRobot);
            obj.m_circles            = [5 5 1];            
            dthetas                  = 0 : 2 * pi / 50 : 2 * pi;
            obj.m_xptsStandardCircle = cos(dthetas);
            obj.m_yptsStandardCircle = sin(dthetas);
        end
    end
    
    properties(GetAccess=public, SetAccess=private)
            m_robotInitVertices;
            m_robotCurrVertices;
            m_robotCurrConfig;     
            m_xptsStandardCircle;
            m_yptsStandardCircle;
    end    
    
    properties(GetAccess=public, SetAccess=public)
        m_circles;
    end

end
