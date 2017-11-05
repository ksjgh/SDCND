#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "spline.h"
#include "helper.h"


// define sensor fusion data access index
// sensor_fusion: data format for each car is: [ id, x, y, vx, vy, s, d]
#define SF_IDX_ID  0 // cf.) float id = sensor_fusion[i][SF_IDX_ID]
#define SF_IDX_X   1 // cf.) float x = sensor_fusion[i][SF_IDX_X]
#define SF_IDX_Y   2 // cf.) float y = sensor_fusion[i][SF_IDX_Y]
#define SF_IDX_VX  3 // cf.) float vx = sensor_fusion[i][SF_IDX_VX]
#define SF_IDX_VY  4 // cf.) float vy = sensor_fusion[i][SF_IDX_VY]
#define SF_IDX_S   5 // cf.) float s = sensor_fusion[i][SF_IDX_S]
#define SF_IDX_D   6 // cf.) float d = sensor_fusion[i][SF_IDX_D]

// total number of points in the path planner, 20ms between points
// N_PATH_POINTS * 20ms = behavior horizon , ex) 50 * 20ms = 1.0[s]
#define N_PATH_POINTS 50
#define DELTA_T 0.02

#define MAX_SPEED_MPH 49.5 // mph
#define MPH_TO_MPS 0.44704 // mile per hour to meter per secound
#define MAX_SPEED (MAX_SPEED_MPH*MPH_TO_MPS)
#define MAX_ACCEL 10 // total acceleration(tangential , centrifugal) 10 m/s^2
#define MAX_JERK 10 //  10 m/s^3

// Weight definition

using namespace std;

#define CAR_STATE_IDX_X     0
#define CAR_STATE_IDX_Y     1
#define CAR_STATE_IDX_S     2
#define CAR_STATE_IDX_D     3
#define CAR_STATE_IDX_YAW   4
#define CAR_STATE_IDX_SPEED 5

#define EPSILON_S (1.0e-2)  // for gradient calculation
#define EPSILON_D (1.0e-2)  // for gradient calculation

#define W_P_GOAL_ZERO 0

double get_potential(VectorXd p,
                     vector<double> car_state,
                     vector<vector<double>> sensor_fusion)
{
  double potential;
  double s = p(0);
  // double car_s = car_state[CAR_STATE_IDX_S];
  double car_s = 0.0;
  double Sg = 100.0;
  potential = -MAX_SPEED*(s-car_s) - W_P_GOAL_ZERO;

  return potential;
}

int main()
{
  double potential;
  VectorXd p(2),ps(2),pd(2);
  p  << 0, 0;
  ps << EPSILON_S, 0;
  pd << 0        , EPSILON_D;

  vector<double> car_state;
  vector<vector<double>> sensor_fusion;

  double p_p  = get_potential(p, car_state,sensor_fusion);
  double p_ps = get_potential(ps,car_state,sensor_fusion);
  double p_pd = get_potential(pd,car_state,sensor_fusion);

  cout << endl;
  cout <<"p_p = " <<  p_p <<endl;
  cout <<"p_ps = " << p_ps <<endl;
  cout <<"p_pd = " << p_pd<<endl;

  double grad_s, grad_d;
  grad_s = (p_ps - p_p)/(double)EPSILON_S;
  grad_d = (p_pd - p_p)/(double)EPSILON_D;

  //debug
  cout << "\n";
  cout << "--------------- get_gradient"<<"\n";
  cout << "MAX_SPEED = "<< MAX_SPEED << endl;
  cout << "grad_s , grad_d = " << grad_s << ","<< grad_d <<"\n";

  cout << endl;
  return 0;
}
