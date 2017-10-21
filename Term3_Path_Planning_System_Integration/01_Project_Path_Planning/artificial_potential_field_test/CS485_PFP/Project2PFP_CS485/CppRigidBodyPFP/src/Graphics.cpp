#include "Graphics.hpp"
#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;


Graphics::Graphics(const char fnameRobot[]) {
    m_simulator.ReadRobot(fnameRobot);    
    m_planner = new RigidBodyPlanner(&m_simulator);

    m_selectedCircle = -1;
    m_editRadius     = false;
    m_run            = false;
    
    
    
}

Graphics::~Graphics(void){
    if(m_planner)
		delete m_planner;
}

void Graphics::MainLoop(void){	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Planner");	   	


//register callback functions
    glutDisplayFunc	( CallbackEventOnDisplay );//-->
    glutMouseFunc	( CallbackEventOnMouse );
    glutMotionFunc	( CallbackEventOnMouseMotion );
    glutIdleFunc	( NULL );
    
    glutTimerFunc	(15, CallbackEventOnTimer, 0);//-->
    glutKeyboardFunc(CallbackEventOnKeyPress);

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void){

        int NrRobots = m_simulator.GetNrRobots();
        
        vector<int> index;
        vector<double> goal_dist;
        double gx = m_simulator.GetGoalCenterX();
        double gy = m_simulator.GetGoalCenterY();
        double x_disp = gx - m_simulator.m_robot.m_x[0];
        double y_disp = gy - m_simulator.m_robot.m_y[0];
        double disp = sqrt(pow(x_disp,2) + pow(y_disp,2));
        
        index.push_back(0);
        goal_dist.push_back(disp);
        
        int insertAt = 0;
        
        for(int i=1; i<NrRobots; i++) //loop starts at index 1 as sub-robot 0 has already been pushed
        {
                x_disp = gx - m_simulator.m_robot.m_x[i];
                y_disp = gy - m_simulator.m_robot.m_y[i];
                disp = sqrt(pow(x_disp,2) + pow(y_disp,2));

                //step through dist_and_index
                //until a value a dist_to_goal larger than disp is found
                //and insert it there
                for(int j=0; j<index.size(); j++)
                {
                        if(goal_dist[i] > disp)
                        {
                                goal_dist.insert(goal_dist.begin()+j, disp);
                                index.insert(index.begin()+j,i);
                                break;
                        } else if(j == index.size() - 1)
                        {
                        	goal_dist.push_back(disp);
                        	index.push_back(i);
                        }
                }
        }
        //new implementation goes in order of distance from goal, shortest to greatest
        for(int i=0; i<index.size(); i++)
        {
                RigidBodyMove move = m_planner->ConfigurationMove(index[i]);
                m_simulator.AddToRobotConfiguration(index[i],move.m_dx, move.m_dy, move.m_dtheta);
        }
        /*
        struct DistAndIndex
        {
                int index;
                double dist_to_goal;
                DistAndIndex(int idx,double dtg) : dist_to_goal(dtg), index(idx) { }
        };
        
        int NrRobots = m_simulator.GetNrRobots();
        
        vector<DistAndIndex> dist_and_index;
        dist_and_index init;
        init.index = 0;
        double gx = m_simulator.GetGoalCenterX();
        double gy = m_simulator.GetGoalCenterY();
        double x_disp = gx - m_simulator.m_simulator.m_robot.m_x[0];
        double y_disp = gy - m_simulator.m_simulator.m_robot.m_y[0];
        double disp = sqrt(pow(x_disp,2) + pow(y_disp,2));
        init.dist_to_goal = disp;
        dist_and_index.push_back(init);
        
        int insertAt = 0;
        
        for(int i=1; i<NrRobots; i++) //loop starts at index 1 as sub-robot 0 has already been pushed
        {
                x_disp = gx - m_simulator.m_simulator.m_robot.m_x[i];
                y_disp = gy - m_simulator.m_simulator.m_robot.m_y[i];
                disp = sqrt(pow(x_disp,2) + pow(y_disp,2));

                //step through dist_and_index
                //until a value a dist_to_goal larger than disp is found
                //and insert it there
                for(vector<DistAndIndex>::iterator it = dist_and_index.begin(); it != dist_and_index.end(); ++it)
                {
                        if(it->dist_to_goal > disp)
                        {
                                dist_and_index.insert(it,new DistAndIndex(disp,i));
                                break;
                        }
                }
        }
        //new implementation goes in order of distance from goal, shortest to greatest
        for(vector<DistAndIndex>::iterator it = dist_and_index.begin(); it != dist_and_index.end(); ++it)
        {
                RigidBodyMove move = m_planner->ConfigurationMove(it->index);
                m_simulator.AddToRobotConfiguration(it->index,move.m_dx, move.m_dy, move.m_dtheta);
        }*/
        //old implementation goes in order of index
        /*/loops thru every sub-robot, computing its next move and then applying the move
        for(int i=0; i<m_simulator.GetNrRobots(); i++)
        {
                if(m_run && !m_simulator.HasRobotReachedGoal())
                {
                        RigidBodyMove move = m_planner->ConfigurationMove(i);
                        m_simulator.AddToRobotConfiguration(i,move.m_dx, move.m_dy, move.m_dtheta);        
                }
        }//*/
}  

void Graphics::HandleEventOnMouseMotion(const double mousePosX, const double mousePosY){
    if(m_selectedCircle >= 0)
    {
	if(m_editRadius)
	{
	    const double cx = m_simulator.m_circles[3 * m_selectedCircle];
	    const double cy = m_simulator.m_circles[3 * m_selectedCircle + 1];
	    
	    m_simulator.m_circles[3 * m_selectedCircle + 2] = sqrt((cx - mousePosX) * (cx - mousePosX) +
								   (cy - mousePosY) * (cy - mousePosY));
	}
	else
	{
	    m_simulator.m_circles[3 * m_selectedCircle] = mousePosX;
	    m_simulator.m_circles[3 * m_selectedCircle + 1] = mousePosY;
	}
	
    }
    
}

void Graphics::HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY){    
    m_selectedCircle = -1;
    for(int i = 0; i < m_simulator.m_circles.size() && m_selectedCircle == -1; i += 3){
	const double cx = m_simulator.m_circles[i];
	const double cy = m_simulator.m_circles[i + 1];
	const double r  = m_simulator.m_circles[i + 2];
	const double d  = sqrt((mousePosX - cx) * (mousePosX - cx) + (mousePosY - cy) * (mousePosY - cy));
	
	if(d <= r)
	    m_selectedCircle = i / 3;
    }
    
    if(m_selectedCircle == -1){
		m_simulator.m_circles.push_back(mousePosX);
		m_simulator.m_circles.push_back(mousePosY);
		m_simulator.m_circles.push_back(1.0);
    }    
}

void Graphics::HandleEventOnKeyPress(const int key){
    printf("pressed key = %d\n", key);
    
    switch(key){
		case 27: //escape key
		exit(0);
	
		case 'r':
		m_editRadius = !m_editRadius;	
		break;
	
		case 'p':
		m_run = !m_run;
		break;
    }
}

//MODIFIED---------------------------<>
void Graphics::HandleEventOnDisplay(void){
	printf("\n->HandleEventOnDisplay");
	
	int n_robots = m_simulator.GetNrRobots();
	printf("\n\tNrRobots: %d\n", n_robots);
	
	//draw robot
    glColor3f(1, 0, 0);
    
    glPolygonMode(GL_FRONT, GL_FILL);	
    
    int ntri;//prints the vertices for the polygon to be drawn
    
    //prints the index of each vertex
    const double *vert; 	const int    *tri;  
    
    for(int r_index=0; r_index<n_robots; r_index++){
		//getss the number of triangles
		ntri = m_simulator.m_robot.m_triangles[r_index].size();
		printf("\n\tNTRI: %d ", ntri);//testing
		
		vert = m_simulator.GetRobotVertices(r_index);
		for (int i=0; i<m_simulator.m_robot.m_currVertices[r_index].size(); i++)	
			printf("\n\t\tVERT: %f ", vert[i] );//testing
		
		printf("\n");
		
		tri = &(m_simulator.m_robot.m_triangles[r_index][0] );
		
		for (int i=0; i<ntri; i++)	printf("\n\t\t->TRI: %d ", tri[i] );//testing
    
		printf("\n\t\tNrOfVertices: %d\n", (int)m_simulator.m_robot.m_currVertices[r_index].size());
		printf("\n=============================================>");
		//exit(0);
		glColor3f(1, 0, 0);
		
		glBegin(GL_TRIANGLES);
				   
				for(int i = 0; i < ntri; i++ )
					glVertex2i(vert[2 * tri[i]], vert[1 + 2 * tri[i]]);
				
		glEnd();
		
		
	}//end of FOR-LOOP -- ObJECT DRAWING
	
	//draw goal and obstacles
	glColor3f(0, 1, 0);
		DrawCircle2D(m_simulator.GetGoalCenterX(), m_simulator.GetGoalCenterY(), m_simulator.GetGoalRadius() );
		glColor3f(0, 0, 1);
		for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
		
		DrawCircle2D(	m_simulator.m_circles[3 + 3 * i], 
						m_simulator.m_circles[4 + 3 * i], 
						m_simulator.m_circles[5 + 3 * i]
					);
}


void Graphics::DrawCircle2D(const double cx, const double cy, const double r){
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
	
    glEnd();	
}


void Graphics::CallbackEventOnDisplay(void){
	printf("\n->CallbackEventOnDisplay");
	//exit(0);
	
    if(m_graphics){
		glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
		glClearDepth(1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
		glEnable(GL_DEPTH_TEST);
		glShadeModel(GL_SMOOTH);	
		
		glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-22, 22, -14, 14, -1.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();	    
		
		m_graphics->HandleEventOnDisplay();
		
		glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y){
    if(m_graphics &&  state == GLUT_DOWN){
		double mouseX, mouseY;
		MousePosition(x, y, &mouseX, &mouseY);
		m_graphics->HandleEventOnMouseBtnDown(button, mouseX , mouseY);
		glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnMouseMotion(int x, int y){
    double mouseX, mouseY;
    MousePosition(x, y, &mouseX, &mouseY);
    m_graphics->HandleEventOnMouseMotion(mouseX , mouseY);
    glutPostRedisplay();
}


void Graphics::CallbackEventOnTimer(int id){
    if(m_graphics){
		m_graphics->HandleEventOnTimer();
		glutTimerFunc(15, CallbackEventOnTimer, id);
		glutPostRedisplay();	    
    }
}



void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y){
    if(m_graphics)
		m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY){
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

int main(int argc, char **argv){
    if(argc < 2){
		printf("missing arguments\n");		
		printf("Planner <robotFile>\n");
		return 0;		
    }

    Graphics graphics(argv[1]);
    
    graphics.MainLoop();
    
    return 0;    
}
