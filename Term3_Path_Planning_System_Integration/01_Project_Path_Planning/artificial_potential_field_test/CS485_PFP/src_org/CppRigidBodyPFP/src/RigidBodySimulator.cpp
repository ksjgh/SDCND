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

#include "RigidBodySimulator.hpp"

RigidBodySimulator::RigidBodySimulator(void)
{
    m_robot.m_x = m_robot.m_y = m_robot.m_theta = 0;    
    m_circles.push_back(16);
    m_circles.push_back(-7);
    m_circles.push_back(1.0);    
}

RigidBodySimulator::~RigidBodySimulator(void)
{
}

Point RigidBodySimulator::ClosestPointOnObstacle(const int i, const double x, const double y)
{
    const double cx = m_circles[3 + 3 * i];
    const double cy = m_circles[4 + 3 * i];
    const double r  = m_circles[5 + 3 * i];
    const double d  = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

    Point p;
    
    p.m_x = cx + r * (x - cx) / d;
    p.m_y = cy + r * (y - cy) / d;

    return p;
}

void RigidBodySimulator::AddToRobotConfiguration(const double dx, const double dy, const double dtheta)
{
    m_robot.m_x += dx;
    m_robot.m_y += dy;
    m_robot.m_theta += dtheta;
    
    const double ctheta = cos(m_robot.m_theta);
    const double stheta = sin(m_robot.m_theta);
    const int    n      = m_robot.m_currVertices.size();
    
    for(int i = 0; i < n; i += 2)
    {
	m_robot.m_currVertices[i] = 
	    ctheta * m_robot.m_initVertices[i] -
	    stheta * m_robot.m_initVertices[i + 1] + m_robot.m_x;
	
	m_robot.m_currVertices[i + 1] = 
	    stheta * m_robot.m_initVertices[i] +
	    ctheta * m_robot.m_initVertices[i + 1] + m_robot.m_y;
    }

}



void RigidBodySimulator::ReadRobot(const char fname[])
{
    FILE *in = fopen(fname, "r");
    int   n  = 0;
    
    if(in)
    {
	if(fscanf(in, "%d", &n) != 1)
	    return;
    
	m_robot.m_currVertices.resize(2 * n);	    
	for(int i = 0; i < 2 * n; ++i)
	    if(fscanf(in, "%lf", &(m_robot.m_currVertices[i])) != 1)
		return;
    
	m_robot.m_triangles.resize(3 * (n - 2));
	for(int i = 0; i < (int) m_robot.m_triangles.size(); ++i)
	    if(fscanf(in, "%d", &(m_robot.m_triangles[i])) != 1)
		return;
	    else
		printf("%d ", m_robot.m_triangles[i]);
	printf("\n...done with tris\n");
	
    
  
	m_robot.m_initVertices.assign(m_robot.m_currVertices.begin(),
				      m_robot.m_currVertices.end());
	fclose(in);	    
    }	
    else
	printf("..could not open file <%s>\n", fname);    
}



