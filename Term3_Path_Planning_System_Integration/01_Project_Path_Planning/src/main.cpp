#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

// define sensor fusion data access index
#define SF_IDX_VX  3 // cf.) float vx = sensor_fusion[i][SF_IDX_D]
#define SF_IDX_VY  4 // cf.) float vy = sensor_fusion[i][SF_IDX_D]
#define SF_IDX_S   5 // cf.) float s = sensor_fusion[i][SF_IDX_D]
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

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // // start in lane 1
  // int lane = 1;
  //
  // // reference velocity to target
  // double ref_vel = 49.5; // mph


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values
          	double previous_path_end_s = j[1]["end_path_s"];
          	double previous_path_end_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            int previous_path_size = previous_path_x.size();
            vector<double> new_pts_x, new_pts_y;
            vector<double> new_pts_s, new_pts_d;

            // /* if previous size is almost empty, use current position
            //    as starting reference */
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
            // // use the previous path's end point as starting reference
            // // to get smooth trasition
            // else
            // {
            //   // redefine reference state as previous path end point
            //   ref_x = previous_path_x[previous_path_size-1];
            //   ref_y = previous_path_y[previous_path_size-1];
            //
            //   double ref_x_prev = previous_path_x[previous_path_size-2];
            //   double ref_y_prev = previous_path_y[previous_path_size-2];
            //   ref_yaw = atan2( ref_y - ref_y_prev, ref_x - ref_x_prev);
            //
            //   // Use tow points that make the path tangent to the previous path's end points
            //   ptsx.push_back(ref_x_prev);
            //   ptsx.push_back(ref_x);
            //
            //   ptsy.push_back(ref_y_prev);
            //   ptsy.push_back(ref_y);
            // }


            //debug
            cout << "--------------- New loop ------------------------"<<endl;
            cout << "previous_path_size = " << previous_path_size <<endl;
            // cout << "previous_path_x.begin() , previous_path_y.begin() = "
            //     //  << previous_path_x.begin() << ","<< previous_path_y.begin() <<endl;
            //      << previous_path_x[0] << ","<< previous_path_y[0] <<endl;
            // cout << "previous_path_x.end() , previous_path_y.end() = "
            //      << previous_path_x[previous_path_size-1] << ","<< previous_path_y[previous_path_size-1] <<endl;
            cout << "car_x , car_y = " << car_x << ","<< car_y <<endl;
            cout << "car_s , car_d = " << car_s << ","<< car_d <<endl;
            //end debug

            VectorXd velocity(2);
            velocity << MAX_SPEED_MPH * MPH_TO_MPS , 0;

            VectorXd next_pos(2);
            VectorXd current_pos(2);
            current_pos << previous_path_end_s,
                           previous_path_end_d;

            double n_rest_points = N_PATH_POINTS - previous_path_size;
            for(int i = 0; i < n_rest_points; i++)
            {
                  next_pos = current_pos +  velocity * DELTA_T;

                  new_pts_s.push_back(next_pos(0));
                  new_pts_d.push_back(next_pos(1));

                  current_pos = next_pos;
            }

            for(int i = 0; i < n_rest_points; i++)
            {
                  vector<double> next_xy(2);
                  next_xy = getXY(new_pts_s[i],
                                  new_pts_d[i],
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y);

                  new_pts_x.push_back(next_xy[0]);
                  new_pts_y.push_back(next_xy[1]);
            }

            for(int i=0; i<previous_path_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            for(int i=0; i<new_pts_x.size(); i++)
            {
              next_x_vals.push_back(new_pts_x[i]);
              next_y_vals.push_back(new_pts_y[i]);
            }

            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
