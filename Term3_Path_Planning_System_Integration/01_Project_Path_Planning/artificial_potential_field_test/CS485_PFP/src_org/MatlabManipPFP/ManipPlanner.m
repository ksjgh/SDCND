% Erion Plaku
% Introduction to Robotics
% Department of Electrical Engineering and Computer Science
% Catholic University of America
%
% http://faculty.cua.edu/plaku/
%
% Copyright 2012 by Erion Plaku
% The content may be reproduced under the obligation to acknowledge the source

function allLinksDeltaTheta = ManipPlanner()
% This is the function that you should implement.
% This function needs to compute by how much the link angles should change
% so that the robot makes a small move toward the goal while avoiding
% obstacles, as guided by the potential field.
%
% allLinksDeltaTheta(j) should contain the small delta change for the angle
% associated with the j-th link.
%
% Note that the attractive potential should be defined only between the end
% effector point on the manipulator and the goal center. 
%
% The repulsive potential on the other hand should be defined between each
% obstacle and each link end.
%
% This will ensure that, when possible, the end effector moves toward the
% goal while every link avoids collisions with obstacles.
%
% You have access to the simulator.
% You can use the methods available in simulator to get all the information
% you need to correctly implement this function
%
    global manipSimulator;
    
    nrLinks        = manipSimulator.GetNrLinks();
    nrObstacles    = manipSimulator.GetNrObstacles();
    [goalx, goaly] = manipSimulator.GetGoalCenter();
    
    allLinksDeltaTheta = zeros(1, nrLinks);
    
    % continue with your code ...
end

