function [dx, dy, dtheta] = RigidBodyPlanner()
% This is the function that you should implement.
% This function needs to compute by how much the position (dx, dy) 
% and orientation (dtheta) should change so that the robot makes a small 
% move toward the goal while avoiding obstacles, 
% as guided by the potential field.
%
% You have access to the simulator.
% You can use the methods available in simulator to get all the information
% you need to correctly implement this function
%

    global rigidBodySimulator;

    [x, y, theta] = rigidBodySimulator.GetRobotCurrentConfig();
    [xgoal, ygoal]= rigidBodySimulator.GetGoalCenter();
    currVertices  = rigidBodySimulator.GetRobotCurrentVertices();
    nrObstacles   = rigidBodySimulator.GetNrObstacles();

    % continue now with your code ...
    
end

