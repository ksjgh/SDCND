% Erion Plaku
% Introduction to Robotics
% Department of Electrical Engineering and Computer Science
% Catholic University of America
%
% http://faculty.cua.edu/plaku/
%
% Copyright 2012 by Erion Plaku
% The content may be reproduced under the obligation to acknowledge the source

function [  ] = RunManipPlanner( nrLinks, linkLength)
% Graphical interface to run planner
% You do not need to modify this file

   global manipSimulator;
   global selectedCircle;
   global mode;
   global shouldRun;
         
   manipSimulator      = ManipSimulator(nrLinks, linkLength);
   selectedCircle = -1;
   mode           = 0;
   shouldRun      = 0;
   
   t = timer('TimerFcn', @MyTimerFcn, 'Period', 0.05, 'ExecutionMode', 'fixedRate', 'BusyMode', 'drop');
   start(t);
   
   clf;
   figure(1);
   set(gcf, 'windowbuttondownfcn', {@MyBtnDownFcn});
   set(gcf, 'windowbuttonupfcn', {@MyBtnUpFcn});
   set(gcf, 'windowbuttonmotionfcn', {@MyBtnMotionFcn});
   set(gcf, 'KeyPressFcn', {@MyKeyPressFcn});

   waitfor(gcf);
   stop(t);  
end

   
function [] = MyTimerFcn(varargin)
   global manipSimulator;
   global shouldRun;
   
    a = manipSimulator.GetSumLinkLengths();
    
    clf;
    set(gca, 'xlim', [-1.1 * a, 1.1 * a]); 
    set(gca, 'ylim', [-1.1 * a, 1.1 * a]);
    grid on;
    hold on;
    if shouldRun == 1 && manipSimulator.HasRobotReachedGoal() == 0
        allLinksDeltaTheta = ManipPlanner();
        manipSimulator = manipSimulator.AddDeltaThetas(allLinksDeltaTheta);
        manipSimulator = manipSimulator.FK();
    end
    manipSimulator.Draw();
end

function [] = MyBtnUpFcn(varargin)
    global selectedCircle;
    selectedCircle = -1;
end

function [] = MyBtnDownFcn(varargin)
    global manipSimulator;
    global selectedCircle;

    cp             = get(gca, 'CurrentPoint');
    nrCircles      = length(manipSimulator.m_circles);    
    selectedCircle = -1;
    for i = 1 : 3: nrCircles
        center = [manipSimulator.m_circles(i), manipSimulator.m_circles(i + 1)];
        if(norm(center - [cp(1, 1), cp(1, 2)]) <= manipSimulator.m_circles(i+2))
            selectedCircle = 1 + fix((i - 1) / 3);
            return;
        end        
    end
    if selectedCircle == -1
        manipSimulator = manipSimulator.AddCircle(cp(1, 1), cp(1, 2), 2);
    end
end

function [] = MyBtnMotionFcn(varargin)
    global manipSimulator;
    global selectedCircle;
    global mode;
    
    if selectedCircle >= 1
        cp = get(gca, 'CurrentPoint');
      
        i = 3 * selectedCircle;
        
        if mode == 1
            manipSimulator.m_circles(i) = norm([manipSimulator.m_circles(i - 2), ...
                                           manipSimulator.m_circles(i-1)] - [cp(1, 1), cp(1, 2)]);
        else
          manipSimulator.m_circles(i - 2) = cp(1, 1);
          manipSimulator.m_circles(i - 1) = cp(1, 2);
        end
    end
end

function [] = MyKeyPressFcn(varargin)
  global mode;
  global shouldRun;
  
  key = get(gcf, 'CurrentCharacter');
  if key == 'r'
      mode = 1;
  elseif key == 'p'
      shouldRun = ~shouldRun;
  else 
      mode = 0;
  end
end
