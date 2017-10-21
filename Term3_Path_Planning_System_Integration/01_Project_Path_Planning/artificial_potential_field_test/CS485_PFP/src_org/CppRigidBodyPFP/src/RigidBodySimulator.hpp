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

#ifndef RIGID_BODY_SIMULATOR_HPP_
#define RIGID_BODY_SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>


struct Point
{
    double m_x;
    double m_y;
};

   
    
class RigidBodySimulator
{
public:    
    RigidBodySimulator(void);
    
    ~RigidBodySimulator(void);

    double GetGoalCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[1];	
    }

    double GetGoalRadius(void) const
    {
	return m_circles[2];
    }
    

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 1;
    }

    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y]
     */
    Point ClosestPointOnObstacle(const int i, const double x, const double y);


    double GetRobotX(void) const
    {
	return m_robot.m_x;
    }
    
    double GetRobotY(void) const
    {
	return m_robot.m_y;
    }
    
    double GetRobotTheta(void) const
    {
	return m_robot.m_theta;
    }

    int GetNrRobotVertices(void) const
    {
	return m_robot.m_currVertices.size() / 2;
    }
    
    /*
     * Vertices are stored consecutively in the vector
     * So the x-coord of the i-th vertex is at index 2 * i
     * and the y-coord of the i-th vertex is at index 2 * i + 1
     */
    const double* GetRobotVertices(void) const
    {
	return &(m_robot.m_currVertices[0]);
    }

    /**
     *@brief Returns true iff the robot center is inside the goal circle
     */
    bool HasRobotReachedGoal(void) const
    {
	const double gx = GetGoalCenterX();
	const double gy = GetGoalCenterY();
	const double rx = GetRobotX();
	const double ry = GetRobotY();
	
	return 
	    sqrt((gx - rx) * (gx - rx) + (gy - ry) * (gy - ry)) <= GetGoalRadius();
    }
    
    
protected:
    void AddToRobotConfiguration(const double dx, const double dy, const double dtheta);

    void ReadRobot(const char fname[]);

    std::vector<double> m_circles;

    struct Robot
    {
	std::vector<double> m_initVertices;
	std::vector<double> m_currVertices;
	std::vector<int>    m_triangles;
	double              m_x;
	double              m_y;
	double              m_theta;
    };
	
    Robot m_robot;

    friend class Graphics;
};

#endif
