#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <math.h>
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

  //debug
  cout << "--------------- Func called ------------------------"<<"\n";
  cout << "previous_path_xy[0].size() = " << previous_path_xy[0].size() <<"\n";
  cout << "previous_path_xy.size() = " << previous_path_xy.size() <<"\n";

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

  VectorXd velocity(2);
  velocity << MAX_SPEED_MPH * MPH_TO_MPS , 0;

  VectorXd p1(2);
  VectorXd p2(2);
  p1 << previous_path_end_s,
        previous_path_end_d;

  double n_rest_points = N_PATH_POINTS - previous_path_size;
  for(int i = 0; i < n_rest_points; i++)
  {

        // get velocity
            // potential_field = get_potential(double target_speed, sensor_fusion)
            // velocity = -get_gradient(p,potential_field)
            // check speed limit , update velocity

        // velocity = get_velocity();

        p2 = p1 +  velocity * DELTA_T;

        // p2 = get_next_point

        new_pts_sd[0].push_back(p2(0));
        new_pts_sd[1].push_back(p2(1));

        // //debug
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
