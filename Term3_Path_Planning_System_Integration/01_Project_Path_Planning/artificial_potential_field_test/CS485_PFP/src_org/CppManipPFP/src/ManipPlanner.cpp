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

#include "ManipPlanner.hpp"

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
   
}

