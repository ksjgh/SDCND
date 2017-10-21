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

#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;

Graphics::Graphics(const char fnameRobot[]) 
{
    m_simulator.ReadRobot(fnameRobot);    
    m_planner = new RigidBodyPlanner(&m_simulator);

    m_selectedCircle = -1;
    m_editRadius     = false;
    m_run            = false;
    
}

Graphics::~Graphics(void)
{
    if(m_planner)
	delete m_planner;
}

void Graphics::MainLoop(void)
{	
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
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutMotionFunc(CallbackEventOnMouseMotion);
    glutIdleFunc(NULL);
    glutTimerFunc(15, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{
    if(m_run && !m_simulator.HasRobotReachedGoal())
    {
	RigidBodyMove move = m_planner->ConfigurationMove();
	m_simulator.AddToRobotConfiguration(move.m_dx, move.m_dy, move.m_dtheta);	
    }
} 

void Graphics::HandleEventOnMouseMotion(const double mousePosX, const double mousePosY)
{
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

void Graphics::HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY)
{    
    m_selectedCircle = -1;
    for(int i = 0; i < m_simulator.m_circles.size() && m_selectedCircle == -1; i += 3)
    {
	const double cx = m_simulator.m_circles[i];
	const double cy = m_simulator.m_circles[i + 1];
	const double r  = m_simulator.m_circles[i + 2];
	const double d  = sqrt((mousePosX - cx) * (mousePosX - cx) + (mousePosY - cy) * (mousePosY - cy));
	
	if(d <= r)
	    m_selectedCircle = i / 3;
    }
    
    if(m_selectedCircle == -1)
    {
	m_simulator.m_circles.push_back(mousePosX);
	m_simulator.m_circles.push_back(mousePosY);
	m_simulator.m_circles.push_back(1.0);
    }    
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    printf("pressed key = %d\n", key);
    
    switch(key)
    {
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

void Graphics::HandleEventOnDisplay(void)
{
//draw robot
    glColor3f(1, 0, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    
    const int     ntri = m_simulator.m_robot.m_triangles.size();
    const double *vert = m_simulator.GetRobotVertices();
    const int    *tri  = &(m_simulator.m_robot.m_triangles[0]);
    
    glColor3f(1, 0, 0);    
    glBegin(GL_TRIANGLES);    
    for(int i = 0; i < ntri; ++i)
	glVertex2d(vert[2 * tri[i]], vert[1 + 2 * tri[i]]);
    
    glEnd();
    
//draw goal and obstacles

    glColor3f(0, 1, 0);
    DrawCircle2D(m_simulator.GetGoalCenterX(), m_simulator.GetGoalCenterY(), m_simulator.GetGoalRadius());
    glColor3f(0, 0, 1);
    for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
	DrawCircle2D(m_simulator.m_circles[3 + 3 * i], 
		     m_simulator.m_circles[4 + 3 * i], 
		     m_simulator.m_circles[5 + 3 * i]);
}


void Graphics::DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}


void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
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

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics &&  state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseBtnDown(button, mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnMouseMotion(int x, int y)
{
    double mouseX, mouseY;
    MousePosition(x, y, &mouseX, &mouseY);
    m_graphics->HandleEventOnMouseMotion(mouseX , mouseY);
    glutPostRedisplay();
}


void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(15, CallbackEventOnTimer, id);
	glutPostRedisplay();	    
    }
}



void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
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

int main(int argc, char **argv)
{
    if(argc < 2)
    {
	printf("missing arguments\n");		
	printf("  Planner <robotFile>\n");
	return 0;		
    }

    Graphics graphics(argv[1]);
    
    graphics.MainLoop();
    
    return 0;    
}
