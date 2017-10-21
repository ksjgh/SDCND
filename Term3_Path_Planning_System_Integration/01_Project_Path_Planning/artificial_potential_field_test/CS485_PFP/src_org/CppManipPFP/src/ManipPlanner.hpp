/**
 * Erion Plaku
 * Introduction to Robotics
 * Department of Electrical Engineering and Computer Science
 * Catholic University of America
 *
 * http://faculty.cua.edu/plaku/
 *
 * Copyright 2012 by Erion Plaku
 * The content may be reproduced under the obligation to acknowledge the source
 */

#ifndef MANIP_PLANNER_HPP_
#define MANIP_PLANNER_HPP_

#include "ManipSimulator.hpp"

class ManipPlanner
{
public:
    ManipPlanner(ManipSimulator * const manipSimulator);
            
    ~ManipPlanner(void);

/*
 * This is the function that you should implement.
 * This function needs to compute by how much the link angles should change
 * so that the robot makes a small move toward the goal while avoiding
 * obstacles, as guided by the potential field.
 *
 * allLinksDeltaTheta(j) should contain the small delta change for the angle
 * associated with the j-th link.
 *
 * Note that the attractive potential should be defined only between the end
 * effector point on the manipulator and the goal center. 
 *
 * The repulsive potential on the other hand should be defined between each
 * obstacle and each link end.
 *
 * This will ensure that, when possible, the end effector moves toward the
 * goal while every link avoids collisions with obstacles.
 *
 * You have access to the simulator.
 * You can use the methods available in simulator to get all the information
 * you need to correctly implement this function
 *
 */
    void ConfigurationMove(double allLinksDeltaTheta[]);
    
        
protected:    
    
    ManipSimulator  *m_manipSimulator;
};

#endif
