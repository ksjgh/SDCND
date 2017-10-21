#include "RigidBodySimulator.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <fstream>
#include <iterator>
#include <algorithm>

int size (char* line);
RigidBodySimulator::RigidBodySimulator(void){
	//must use pushback-> insert single sub-robot configuration into super-robot at default configuration
	
	/*for now...*/
    //m_robot.m_x.push_back(0);
    //m_robot.m_y.push_back(0); 
    //m_robot.m_theta.push_back(0);    
    
    m_circles.push_back(16);
    m_circles.push_back(-7);
    m_circles.push_back(1.0);  
}

RigidBodySimulator::RigidBodySimulator(int number_of_bots){
        //change to initialize every robot at a different location, positioned across the workspace at a uniform-random distribution
        /*NOTE: number of robots not known til ReadRobot is called,
        must create a way to find it by time of or on constructor call*/
        for(int i=0; i<number_of_bots; i++){
                //workspace goes from (-22,-14) on bottom left to (22,14) on top right
                m_robot.m_x.push_back((int)pow(-1,rand()%2)*rand()%22); /*rand using x-range coordinates -22~22*/
                m_robot.m_y.push_back((int)pow(-1,rand()%2)*rand()%14); /*rand using y-range coordinates* -14~14*/
                /*should replace rand() above with function that returns a float/double instead of int for better implementation*/
                m_robot.m_theta.push_back(0);
                
                /**WILL ALSO NEED TO CHECK EACH NEW SUB-ROBOT FOR COLLISION BEFORE ADDING IT TO SUPER-ROBOT
                PROBABLY GENERATE PARAMETERS FIRST, CHECK FOR COLLISION, THEN
                IF NO COLLISION -> PUSH_BACK PARAMETERS, ELSE DO NOTHING*/
        }
        m_circles.push_back(16); //goal x
        m_circles.push_back(-7); //goal y
        m_circles.push_back(1.0); //goal radius
}

RigidBodySimulator::~RigidBodySimulator(void){}

Point RigidBodySimulator::ClosestPointOnObstacle(const int i, const double x, const double y){
    const double cx = m_circles[3 + 3 * i];
    const double cy = m_circles[4 + 3 * i];
    const double r  = m_circles[5 + 3 * i];
    const double d  = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

    Point p;
    
    p.m_x = cx + r * (x - cx) / d;
    p.m_y = cy + r * (y - cy) / d;

    return p;
}

void RigidBodySimulator::AddToRobotConfiguration(int r_index, const double dx, const double dy, const double dtheta){
    m_robot.m_x[r_index] += dx;
    m_robot.m_y[r_index] += dy;
    m_robot.m_theta[r_index] += dtheta;
    
    const double ctheta = cos(m_robot.m_theta[r_index]);
    const double stheta = sin(m_robot.m_theta[r_index]);
    const int    n      = m_robot.m_currVertices[r_index].size();
    //const int robots = m_robot.m_
    
    for(int i = 0; i < n; i += 2){
		
		m_robot.m_currVertices[r_index][i] = 
			ctheta * m_robot.m_initVertices[r_index][i] -
			stheta * m_robot.m_initVertices[r_index][i + 1] + m_robot.m_x[r_index];
		
		m_robot.m_currVertices[r_index][i + 1] = 
			stheta * m_robot.m_initVertices[r_index][i] +
			ctheta * m_robot.m_initVertices[r_index][i + 1] + m_robot.m_y[r_index];
    }
}


void RigidBodySimulator::ReadRobot(const char fname[]){
    FILE *in = fopen(fname, "r");
    
    int n  = 0, num = 0, i=0;
    
    if(in){
		// n -should print the number of vertices read 
		
		if(fscanf(in, "%d", &n ) != 1)
			return;
		// n, stores the number of robots
		
		//for-loop stores x,y,theta values for each robot
		for(int xyt=0; xyt<n; xyt++){
	                //workspace goes from (-22,-14) on bottom left to (22,14) on top right
	                m_robot.m_x.push_back((int)pow(-1,rand()%2)*rand()%22); /*rand using x-range coordinates -22~22*/
	                m_robot.m_y.push_back((int)pow(-1,rand()%2)*rand()%14); /*rand using y-range coordinates* -14~14*/
	                /*should replace rand() above with function that returns a float/double instead of int for better implementation*/
	                m_robot.m_theta.push_back(0);
	                
	                /**WILL ALSO NEED TO CHECK EACH NEW SUB-ROBOT FOR COLLISION BEFORE ADDING IT TO SUPER-ROBOT
	                PROBABLY GENERATE PARAMETERS FIRST, CHECK FOR COLLISION, THEN
	                IF NO COLLISION -> PUSH_BACK PARAMETERS, ELSE DO NOTHING*/
		}
		
		printf("\nNumber of Robots: ===========================> <%d>", n);
		int line[n]; //will store all vertices of the total #-of robots
		
		for(int v=0; v<n; v++)	line[v] = 0;
		//gets the line of vertices
		for (int m=0; m<n; m++){
			fscanf(in, "%d", &num) ; 
			line[m] = num;	
			printf("\nRobot <%d> has <%d> Vertices", m, line[m]);;
		}
		
		m_robot.m_currVertices.resize(n);
		m_robot.m_triangles.resize(n );
		m_robot.m_initVertices.resize(n);
		
		for (int j=0; j<n; j++){			
			m_robot.m_currVertices[j].resize(2 * line[j]);
			
			printf("\nNumber of vertices <%d> for Robot: %d", line[j], j);       
			
			for(i = 0; i < 2 * line[j]; i++){
				if(fscanf(in, "%lf", &(m_robot.m_currVertices[j][i])) != 1)	
					return;
					
				//printf("\n%f -->", m_robot.m_currVertices[j][11] );
				printf("\nRobot: %d - i:%d %f",j,i, m_robot.m_currVertices[j][i] );
			}
			
			m_robot.m_triangles[j].resize(3 * (line[j] - 2) );
			
			printf("\nNumber of Triangles: %d for Robot: %d\n",(int) m_robot.m_triangles[j].size(), j);
			
			for(int k = 0; k <   (int)m_robot.m_triangles[j].size(); ++k)
				if(fscanf(in,"%lf", &(m_robot.m_triangles[j][k]) ) != 1)
					return;
				
				else
					printf("%d ", (int)m_robot.m_triangles[j][k]);
			
			printf("\n\n");
			
			 m_robot.m_initVertices[j].assign(	
									m_robot.m_currVertices[j].begin(),
									m_robot.m_currVertices[j].end());
			
			
		}// end of FOR-LOOP
		
        fclose(in);     
        
    }        
    else 	printf("..could not open file <%s>\n", fname);    
}//end of READ-ROBOT
