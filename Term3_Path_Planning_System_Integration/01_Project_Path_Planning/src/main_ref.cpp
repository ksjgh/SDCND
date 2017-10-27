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

#define MAX_SPEED 49.5 // mph

#define SAFE_FRONT_GAP 30.0 // safe distance to front car


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// start in lane 1
int lane = 1;

// reference velocity to target
double ref_vel = 0; // mph

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

        	// ego car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s, d values
          	double previous_path_end_s = j[1]["end_path_s"];
          	double previous_path_end_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_path_x;
          	vector<double> next_path_y;
            int previous_path_size = previous_path_x.size();

            /* start of path planning */
            //------------------------------------------------------------------

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // // start at lane 1
            int lane = 1;

            //------------------------------------------------------------------
            // check for collision with front car
            //------------------------------------------------------------------

            if(previous_path_size > 0)
            {
              car_s = previous_path_end_s;
            }

            bool too_close = false;
            for(int i=0; i < sensor_fusion.size(); i++)
            {
              // find car in my lane
              float d = sensor_fusion[i][SF_IDX_D];
              if( d < (2+4*lane+2) && d > (2+4*lane-2) )
              {
                double vx = sensor_fusion[i][SF_IDX_VX];
                double vy = sensor_fusion[i][SF_IDX_VY];
                double other_car_speed = sqrt(vx*vx + vy*vy);
                double other_car_s = sensor_fusion[i][SF_IDX_S];
                double other_car_d = sensor_fusion[i][SF_IDX_D];

                // predict other car's postion at behavior horizon
                other_car_s += ( (double)previous_path_size*0.02*other_car_speed );

                double front_s_gap = other_car_s - car_s;

                //debug
                cout << "front_s_gap = " << front_s_gap << endl;
                //debug end

                if( (front_s_gap > 0) && ( front_s_gap < (double)SAFE_FRONT_GAP) )
                {
                  // ref_vel = 29.5; // mph
                  too_close = true;
                  // if(lane > 0)
                  // {
                  //   lane = 0;
                  // }
                }

              }
            }

            if(too_close)
            {
              ref_vel -= 0.224; // decrease 1 [m/s]
            }
            else if(ref_vel < MAX_SPEED)
            {
              ref_vel += 0.224;// increase 1 [m/s]
            }

            //------------------------------------------------------------------
            // check for collision end
            //------------------------------------------------------------------


            // -----------------------------------------------------------------
            // Create al list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we will interplolate these waypoints with a spline and ffill it int with more points thate constrol spline

            vector<double> ptsx, ptsy;

            // we will reference the starting point as where the car is or at the previous paths end point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // //debug
            // cout << "--------------- New loop ------------------------"<<endl;
            // cout << "previous_path_x.size = " << previous_path_size <<endl;
            // cout << "car_x , car_y = " << car_x << ","<< car_y <<endl;
            // cout << "previous_path_x.begin() , previous_path_y.begin() = "
            //     //  << previous_path_x.begin() << ","<< previous_path_y.begin() <<endl;
            //      << previous_path_x[0] << ","<< previous_path_y[0] <<endl;
            // cout << "previous_path_x.end() , previous_path_y.end() = "
            //      << previous_path_x[previous_path_size-1] << ","<< previous_path_y[previous_path_size-1] <<endl;
            // //end debug

            /* if previous size is almost empty, use current position
               as starting reference */
            if(previous_path_size < 2)
            {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);

            }
            // use the previous path's end point as starting reference
            // to get smooth trasition
            else
            {
              //debug
              cout << "--------------- New loop ------------------------"<<endl;
              cout << "previous_path_x.size = " << previous_path_size <<endl;
              cout << "car_x , car_y = " << car_x << ","<< car_y <<endl;
              cout << "previous_path_x.begin() , previous_path_y.begin() = "
                  //  << previous_path_x.begin() << ","<< previous_path_y.begin() <<endl;
                   << previous_path_x[0] << ","<< previous_path_y[0] <<endl;
              cout << "previous_path_x.end() , previous_path_y.end() = "
                   << previous_path_x[previous_path_size-1] << ","<< previous_path_y[previous_path_size-1] <<endl;
              //end debug

              // redefine reference state as previous path end point
              ref_x = previous_path_x[previous_path_size-1];
              ref_y = previous_path_y[previous_path_size-1];

              double ref_x_prev = previous_path_x[previous_path_size-2];
              double ref_y_prev = previous_path_y[previous_path_size-2];
              ref_yaw = atan2( ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use tow points that make the path tangent to the previous path's end points
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            //In Frenet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // convert global coordinates into car's local coordinates for easy math
            for(int i=0; i < ptsx.size(); i++)
            {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }

            // create a spline
            tk::spline s;

            // set(x,y) points to the spline ( check if `ptsx` sorted )
            s.set_points(ptsx,ptsy);

            // next x,y values =  previous path x,y + new points(ptsx,y)
            for(int i=0; i < previous_path_size; i++)
            {
              next_path_x.push_back(previous_path_x[i]);
              next_path_y.push_back(previous_path_y[i]);

            }

            // calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x); // cf.) s.set_points(ptsx,ptsy);
            double target_dist = distance(target_x,target_y,0,0);
            double N = (target_dist/(.02 * ref_vel/2.24)); // 2.24 [mph]->[m/s]
            double x_delta = target_x/N;
            double x_prev = 0;

            // Fill up the rest of our path Planner after filling it with previous points
            int n_rest_points = N_PATH_POINTS - previous_path_size;
            for(int i=0; i < n_rest_points; i++)
            {
              // get each (x,y) point in spline
              double x_point = x_prev + x_delta;
              double y_point = s(x_point);

              x_prev = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate & shift , for back to global coordinates
              x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              // append ponit to trajectory
              next_path_x.push_back(x_point);
              next_path_y.push_back(y_point);
            }

            //debug
            cout << "next_path_x.size = " << next_path_x.size() << endl;
            //end debug

            msgJson["next_x"] = next_path_x;
          	msgJson["next_y"] = next_path_y;

            //------------------------------------------------------------------
            /* end of path planning */


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



  //----------------------------------------------------------------------------
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
