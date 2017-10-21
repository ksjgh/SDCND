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
    
    //new method to return no. of robots for gradient calculations
    // +when robots.size = 0, goal reached
    int GetNrRobots(void) const
    {
    	return m_robot.m_theta.size(); //can also use m_x.size() or m_y.size()
    }

    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y]
     */
    Point ClosestPointOnObstacle(const int i, const double x, const double y);
    Point ClosestPointOnRobotRadius(const int i, const double x, const double y);
    
    bool NotInCollision(int x1, int y1, int r1, int x2, int y2, int r2)
    {
    	double distance = sqrt(pow(x2 - x1,2) + pow(y2 - y1,2));
    	
    	return distance >= (r2 + r1);
    }
    
    Point GetCentroid(int i)
    {
    	double A = 0;
    	double x1,y1,x2,y2;
    	for(int j=0; j<m_robot.m_currVertices[i].size(); j+=2)
    	{
    		x1 = m_robot.m_currVertices[i][j];
    		y1 = m_robot.m_currVertices[i][j+1];
    		x2 = m_robot.m_currVertices[i][j+2];
    		y2 = m_robot.m_currVertices[i][j+3]
    		A += 0.5*(x1*y2 - x2*y1);
    	}
    	double sumX,sumY;
    	sumX = sumY = 0;
    	for(int j=0; j<m_robot.m_currVertices[i].size(); j+=2)
    	{
    		sumX += (x1 + x2)*(x1*y2 - x2*y1);
    		sumY += (y1 + y2)*(x1*y2 - x2*y1);
    	}
    	
    	return Point(sumX/(6.0*A),sumY/(6.0*A));
    }
    
    double GetRadius(int i, Point C) //calculates smallest radius of a circle centered at centroid such that the sub-robot is bound by the circle
    {
    	double radius;
    	
    	for(int j=0;j<m_robot.currVertices[i].size();j++)
    	{
    		double new_distance = sqrt(pow(C.x - m.robot.currVertices[i][j],2) + pow(C.y - m.robot.currVertices[i][j+1],2));
    		if(new_distance < radius)
    		{
    			radius = new_distance;
    		}
    	}
    	
    	return radius;
    }

    double GetRobotX(int i) const
    {
	return m_robot.m_x[i];
    }
    
    double GetRobotY(int i) const
    {
	return m_robot.m_y[i];
    }
    
    double GetRobotTheta(int i) const
    {
	return m_robot.m_theta[i];
    }

    int GetNrRobotVertices(int i) const
    {
	return m_robot.m_currVertices[i].size() / 2;
    }
    
    /*
     * Vertices are stored consecutively in the vector
     * So the x-coord of the i-th vertex is at index 2 * i
     * and the y-coord of the i-th vertex is at index 2 * i + 1
     */
    const double* GetRobotVertices(int i) const
    {
	return &(m_robot.m_currVertices[i][0]);
    }

    /**
     *@brief Returns true iff the robot center is inside the goal circle
     */
    bool HasRobotReachedGoal(int i) const
    {
	const double gx = GetGoalCenterX();
	const double gy = GetGoalCenterY();
	const double rx = GetRobotX(i);
	const double ry = GetRobotY(i);
	
	return 
	    sqrt((gx - rx) * (gx - rx) + (gy - ry) * (gy - ry)) <= GetGoalRadius();
    }
    
    
protected:
    void AddToRobotConfiguration(int r, const double dx, const double dy, const double dtheta);

    void ReadRobot(const char fname[]);

    std::vector<double> m_circles;

    struct Robot //now defines a super-robot structure
    {
	std::vector<std::vector<double>> m_initVertices;
	std::vector<std::vector<double>> m_currVertices;
	std::vector<std::vector<int>>    m_triangles;
	std::vector<double> m_x;
	std::vector<double> m_y;
	std::vector<double> m_theta;
	std::vector<double> m_totalSolveTime;
    };
	
    Robot m_robot;

    friend class Graphics;
};

#endif
