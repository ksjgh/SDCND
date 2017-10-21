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

#ifndef MANIP_SIMULATOR_HPP_
#define MANIP_SIMULATOR_HPP_

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
    

class ManipSimulator
{
public:    
    ManipSimulator(void);
    
    ~ManipSimulator(void);
    
    double GetGoalCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[1];	
    }

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 1;
    }

    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y]
     */
    Point ClosestPointOnObstacle(const int i, const double x, const double y);

    int GetNrLinks(void) const
    {
	return m_joints.size();
    }

    double GetLinkStartX(const int i) const
    {
	return m_positions[2 * i];
    }

    double GetLinkStartY(const int i) const
    {
	return m_positions[2 * i + 1];
    }

    double GetLinkEndX(const int i) const
    {
	return GetLinkStartX(i + 1);
    }

    double GetLinkEndY(const int i) const
    {
	return GetLinkStartY(i + 1);
    }

    bool HasRobotReachedGoal(void) const;
    
protected:

    double GetObstacleCenterX(const int i) const
    {
	return m_circles[3 * i + 3];
    }
    
    double GetObstacleCenterY(const int i) const
    {
	return m_circles[3 * i + 4];
    }
    
    double GetObstacleRadius(const int i) const
    {
	return m_circles[3 * i + 5];
    }

    double GetLinkTheta(const int i) const
    {
	return m_joints[i];
    }

    double GetLinkLength(const int i) const
    {
	return m_lengths[i];
    }

    double GetGoalRadius(void) const
    {
	return m_circles[2];
    }
    
    void FK(void);


protected:

    void AddLink(const double length);
    
    void AddToLinkTheta(const int i, const double dtheta)
    {
	m_joints[i] += dtheta;
    }

    std::vector<double> m_joints;
    std::vector<double> m_lengths;
    std::vector<double> m_positions;
    std::vector<double> m_circles;
    
    friend class Graphics;
};

#endif
