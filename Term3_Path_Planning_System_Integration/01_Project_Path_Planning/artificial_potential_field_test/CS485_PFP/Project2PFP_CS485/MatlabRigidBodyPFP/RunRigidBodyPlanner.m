function [] = RunRigidBodyPlanner(fnameRobot)
% RunRigidBodyPlanner(fnameRobot)
%   fnameRobot: name of robot file
% Function provides graphical interface to run potential path planner in
% the case of a polygonal rigid body that can translate and rotate in a 2D
% workspace. 
%
% You do not need to make changes to this file.
%
   global rigidBodySimulator;
   global selectedCircle;
   global mode;
   global shouldRun;

   rigidBodySimulator = RigidBodySimulator(fnameRobot);
   rigidBodySimulator.Draw();
      
   fprintf('click to enter robot center...\n');
   center = ginput(1);
   rigidBodySimulator = rigidBodySimulator.AddToRobotConfig(center(1), center(2), 0);
   
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
   global rigidBodySimulator;
   global shouldRun;
       
    if shouldRun == 1 && rigidBodySimulator.HasRobotReachedGoal() == 0
        [dx, dy, dtheta] = RigidBodyPlanner();
        rigidBodySimulator = rigidBodySimulator.AddToRobotConfig(dx, dy, dtheta);
    end
    rigidBodySimulator.Draw();
end

function [] = MyBtnUpFcn(varargin)
    global selectedCircle;
    selectedCircle = -1;
end

function [] = MyBtnDownFcn(varargin)
    global rigidBodySimulator;
    global selectedCircle;

    cp             = get(gca, 'CurrentPoint');
    nrCircles      = length(rigidBodySimulator.m_circles);    
    selectedCircle = -1;
    for i = 1 : 3: nrCircles
        center = [rigidBodySimulator.m_circles(i), rigidBodySimulator.m_circles(i + 1)];
        if(norm(center - [cp(1, 1), cp(1, 2)]) <= rigidBodySimulator.m_circles(i+2))
            selectedCircle = 1 + fix((i - 1) / 3);
            return;
        end        
    end
    if selectedCircle == -1
        rigidBodySimulator = rigidBodySimulator.AddCircle(cp(1, 1), cp(1, 2), 2);
    end
end

function [] = MyBtnMotionFcn(varargin)
    global rigidBodySimulator;
    global selectedCircle;
    global mode;
    
    if selectedCircle >= 1
        cp = get(gca, 'CurrentPoint');
      
        i = 3 * selectedCircle;
        
        if mode == 1
            rigidBodySimulator.m_circles(i) = norm([rigidBodySimulator.m_circles(i - 2), ...
                                           rigidBodySimulator.m_circles(i-1)] - [cp(1, 1), cp(1, 2)]);
        else
          rigidBodySimulator.m_circles(i - 2) = cp(1, 1);
          rigidBodySimulator.m_circles(i - 1) = cp(1, 2);
        end
    end
end

function [] = MyKeyPressFcn(varargin)
  global mode;
  global shouldRun;
  
  key = get(gcf, 'CurrentCharacter');
  if key == 'r'
      mode = ~mode;
  elseif key == 'p'
      shouldRun = ~shouldRun;
  end
end