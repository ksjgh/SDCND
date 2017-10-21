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

#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"

struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);
    
protected:
    RigidBodySimulator *m_simulator;
};

#endif
