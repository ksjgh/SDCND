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

#include "ManipSimulator.hpp"

ManipSimulator::ManipSimulator(void)
{
    m_positions.push_back(0);
    m_positions.push_back(0);

    m_circles.push_back(16);
    m_circles.push_back(-7);
    m_circles.push_back(1.0);    
}

ManipSimulator::~ManipSimulator(void)
{
}

bool ManipSimulator::HasRobotReachedGoal(void) const
{
    const double ex = GetLinkEndX(GetNrLinks() - 1);
    const double ey = GetLinkEndY(GetNrLinks() - 1);
    
    return
	sqrt((ex - GetGoalCenterX()) * (ex - GetGoalCenterX()) +
	     (ey - GetGoalCenterY()) * (ey - GetGoalCenterY())) < GetGoalRadius();
}

void ManipSimulator::AddLink(const double length)
{
    m_joints.push_back(0);
    m_lengths.push_back(length);
    m_positions.resize(m_positions.size() + 2);	
}

Point ManipSimulator::ClosestPointOnObstacle(const int i, const double x, const double y)
{
    const double cx = GetObstacleCenterX(i);
    const double cy = GetObstacleCenterY(i);
    const double r  = GetObstacleRadius(i);
    const double d  = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

    Point p;
    
    p.m_x = cx + r * (x - cx) / d;
    p.m_y = cy + r * (y - cy) / d;

    return p;
    
}

void MatrixMultMatrix(const double M1[6], const double M2[6], double M[6])
{
    double Mresult[6];

    Mresult[0] = M1[0] * M2[0] + M1[1] * M2[3];
    Mresult[1] = M1[0] * M2[1] + M1[1] * M2[4];
    Mresult[2] = M1[0] * M2[2] + M1[1] * M2[5] + M1[2];

    Mresult[3] = M1[3] * M2[0] + M1[4] * M2[3];
    Mresult[4] = M1[3] * M2[1] + M1[4] * M2[4];
    Mresult[5] = M1[3] * M2[2] + M1[4] * M2[5] + M1[5];
    
    M[0] = Mresult[0];
    M[1] = Mresult[1];
    M[2] = Mresult[2];
    M[3] = Mresult[3];
    M[4] = Mresult[4];
    M[5] = Mresult[5];
}

void ManipSimulator::FK(void)
{
    const int n = GetNrLinks();
    
    double M[6];
    double Mall[6];
    double p[2] = {0, 0};
    double pnew[2];
    
    
    Mall[0] = Mall[4] = 1;
    Mall[1] = Mall[2] = Mall[3] = Mall[5] = 0;
    
    for(int i = 0; i < n; ++i)
    {
	const double ctheta = cos(GetLinkTheta(i));
	const double stheta = sin(GetLinkTheta(i));
	
	M[0] = ctheta;  M[1] = -stheta; M[2] = GetLinkLength(i) * ctheta;
	M[3] = stheta;  M[4] =  ctheta; M[5] = GetLinkLength(i) * stheta;

	MatrixMultMatrix(Mall, M, Mall);

	m_positions[2 * i + 2] = Mall[2];
	m_positions[2 * i + 3] = Mall[5];
    }
}

