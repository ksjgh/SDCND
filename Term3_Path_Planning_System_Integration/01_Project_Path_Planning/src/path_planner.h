#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

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

#define CAR_STATE_IDX_X     0
#define CAR_STATE_IDX_Y     1
#define CAR_STATE_IDX_S     2
#define CAR_STATE_IDX_D     3
#define CAR_STATE_IDX_YAW   4
#define CAR_STATE_IDX_SPEED 5

#define EPSILON_S (1.0e-2)  // for gradient calculation
#define EPSILON_D (1.0e-2)  // for gradient calculation

// total number of points in the path planner, 20ms between points
// N_PATH_POINTS * 20ms = behavior horizon , ex) 50 * 20ms = 1.0[s]
#define N_PATH_POINTS 30
#define DELTA_T 0.02

#define MAX_SPEED_MPH 49.5 // mph
#define MPH_TO_MPS 0.44704 // mile per hour to meter per secound
#define MAX_SPEED ((MAX_SPEED_MPH)*(MPH_TO_MPS))
#define MAX_ACCEL 10 // total acceleration(tangential , centrifugal) 10 m/s^2
#define MAX_JERK 10 //  10 m/s^3

#define LANE_WIDTH 4.0
#define LANE0_CENTER 2.0
#define LANE0_END 4.0
#define LANE1_CENTER 6.0
#define LANE1_END 8.0
#define LANE2_CENTER 10.0
#define LANE2_END 12.0
#define RIGHTMOST 12.0
#define FRONT_SAFE_DIST 30.0

#define W_P_GOAL_BIAS 0.0
#define W_P_CENTER_LANE 5.0
#define W_P_RIGHTMOST_LANE 5.0

#define W_P_OTHER_CAR 10.0
#define W_P_SIGMA_S (FRONT_SAFE_DIST/2.0)
#define W_P_SIGMA_D (LANE_WIDTH/1.0)

using namespace std;

double get_potential(VectorXd p,
                     vector<double> car_state,
                     vector<vector<double>> sensor_fusion)
{
  double p_total = 0.0; // total potential

  double s = p(0);
  double d = p(1);

  double car_s = car_state[CAR_STATE_IDX_S];
  double car_d = car_state[CAR_STATE_IDX_D];

  // free state(no object) potential
  // car runs with maximum speed
  double p_free = -MAX_SPEED*(s-car_s) - W_P_GOAL_BIAS;
  p_total += p_free;

  // center line potential
  double p_center_lane;
  if( d < 0 || d > LANE0_CENTER)
  {
    p_center_lane = 0.0;
  }
  else
  {
      p_center_lane = W_P_CENTER_LANE*(1/d - 1/(LANE_WIDTH/2.0));
  }
  p_total += p_center_lane;

  // rightmost line potential
  double p_rightmost_line;
  if( d < LANE2_CENTER)
  {
    p_rightmost_line = 0.0;
  }
  else
  {
    p_rightmost_line = W_P_RIGHTMOST_LANE*(1/(-(d-RIGHTMOST)) - 1/(LANE_WIDTH/2.0));
  }
  p_total += p_rightmost_line;

  // other car's potential
  for(int i=0; i < sensor_fusion.size(); i++)
  {
    // get other car's data
    // double o_vx = sensor_fusion[i][SF_IDX_VX];
    // double o_vy = sensor_fusion[i][SF_IDX_VY];
    // double o_speed = sqrt(vx*vx + vy*vy);
    double oc_s = sensor_fusion[i][SF_IDX_S];
    double oc_d = sensor_fusion[i][SF_IDX_D];

    double p_oc;
    if( ((oc_s - s) > FRONT_SAFE_DIST) || (oc_s < s) )
    {
      p_oc = 0.0;
    }
    else
    {
      p_oc = W_P_OTHER_CAR * exp(-pow(s-oc_s,2)/pow(W_P_SIGMA_S,2))
                           * exp(-pow(d-oc_d,2)/pow(W_P_SIGMA_D,2));
    }
    p_total += p_oc;
  }

  // lane line potential

  return p_total;
}

VectorXd get_gradient(VectorXd p,
                      vector<double> car_state,
                      vector<vector<double>> sensor_fusion)
{
  VectorXd grad(2);
  VectorXd ps(2);
  VectorXd pd(2);

  ps << p(0)+EPSILON_S, p(1);
  pd << p(0)          , p(1)+EPSILON_D;

  double p_p  = get_potential(p  ,car_state,sensor_fusion);
  double p_ps = get_potential(ps ,car_state,sensor_fusion);
  double p_pd = get_potential(pd ,car_state,sensor_fusion);

  // // debug
  // double p_p0  = 0.0;
  // double p_ps  = -MAX_SPEED*EPSILON_S;
  // double p_pd = 0.0;
  // // debug

  double grad_s, grad_d;
  grad_s = (p_ps - p_p)/(double)EPSILON_S;
  grad_d = (p_pd - p_p)/(double)EPSILON_D;

  grad << grad_s, grad_d;

  // //debug
  // cout << "\n";
  // cout << "--------------- get_gradient"<<"\n";
  // cout << "grad_s , grad_d = " << grad_s << " , "<< grad_d <<"\n";
  // //end debug

  return grad;
}

vector<vector<double>> get_new_path(vector<double> car_state,
                                    vector<vector<double>> sensor_fusion,
                                    vector<vector<double>> previous_path_xy,
                                    vector<double> map_x,
                                    vector<double> map_y)
{
  double car_x = car_state[CAR_STATE_IDX_X];
  double car_y = car_state[CAR_STATE_IDX_Y];
  double car_s = car_state[CAR_STATE_IDX_S];
  double car_d = car_state[CAR_STATE_IDX_D];
  double car_yaw = car_state[CAR_STATE_IDX_YAW];
  double car_speed = car_state[CAR_STATE_IDX_SPEED];

  // vector<double> maps_x = map_waypoints[0];
  // vector<double> maps_y = map_waypoints[1];

  int previous_path_size = previous_path_xy[0].size();
  //
  // //debug
  // cout << "\n";
  // cout << "--------------- Func called ------------------------"<<"\n";
  // cout << "previous_path_xy[0].size() = " << previous_path_xy[0].size() <<"\n";
  // // cout << "previous_path_xy.size() = " << previous_path_xy.size() <<"\n";

  // cout << "car_x , car_y = " << car_x << ","<< car_y <<"\n";
  // cout << "car_s , car_d = " << car_s << ","<< car_d <<"\n";
  //end debug

  vector<vector<double>> new_pts_sd(2);

  double previous_path_end_s;
  double previous_path_end_d;
  if(previous_path_size < 2)
  {
    previous_path_end_s = car_s;
    previous_path_end_d = car_d;

    // // Use two points that make the path tangent to the car
    // double prev_car_s = car_s - 1.0;
    // double prev_car_d = car_d
    //
    // new_pts_s.push_back(prev_car_s);
    // new_pts_s.push_back(car_s);
    //
    // new_pts_d.push_back(prev_car_d);
    // new_pts_d.push_back(car_d);
  }
  else
  {
    double previous_path_end_x = previous_path_xy[0].back();
    double previous_path_end_y = previous_path_xy[1].back();
    double theta = atan2(previous_path_end_y,previous_path_end_x);

    vector<double> previous_path_end_sd;
    previous_path_end_sd = getFrenet(previous_path_end_x,
                                     previous_path_end_y,
                                     theta,
                                     map_x,
                                     map_y);

    previous_path_end_s = previous_path_end_sd[0];
    previous_path_end_d = previous_path_end_sd[1];
  }

  VectorXd p1(2);
  VectorXd p2(2);
  p1 << previous_path_end_s,
        previous_path_end_d;

  double n_rest_points = N_PATH_POINTS - previous_path_size;
  for(int i = 0; i < n_rest_points; i++)
  {

        VectorXd velocity(2);
        velocity = -get_gradient(p1, car_state, sensor_fusion);

        // // debug
        // velocity << MAX_SPEED_MPH * MPH_TO_MPS , 0;
        // // debug

        p2 = p1 +  velocity * DELTA_T;

        new_pts_sd[0].push_back(p2(0));
        new_pts_sd[1].push_back(p2(1));

        // //debug
        // // if this code on, car will go and stop due to delay
        // cout << endl;
        // cout << "new_pts_sd push p2" <<endl;
        // cout << "new_pts_sd[0].back() = " << new_pts_sd[0].back() << endl;
        // cout << "new_pts_sd[1].back() = " << new_pts_sd[1].back() << endl;
        // cout << endl;
        // //end debug

        p1 = p2;
  }

  // //debug
  // cout << "Inside func" <<endl;
  // cout << "new_pts_sd[0].size() = " << new_pts_sd[0].size() <<endl;
  // //end debug

  return new_pts_sd;
}


#endif /* PATH_PLANNER_H */
