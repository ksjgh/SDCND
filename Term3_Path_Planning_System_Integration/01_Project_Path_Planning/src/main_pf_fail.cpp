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

#define MAX_SPEED 49.5 // mph
#define MAX_ACCEL 10 // total acceleration(tangential , centrifugal) 10 m/s^2
#define MAX_JERK 10 //  10 m/s^3


// Gain definition


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

        	// get ego car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Remained part of previous path data
            // Head portion of original previous path points are used by ego car
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s, d values
          	double previous_path_end_s = j[1]["end_path_s"];
          	double previous_path_end_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            /* start of path planning */
            /*****************************************************************/
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // next_path_x,y = previous_path_x,y + new_path_x,y
            vector<double> next_path_x , next_path_y;
          	vector<double> new_path_x, new_path_y;
            int previous_path_size = previous_path_x.size();


            /* compute new_path */

            // set starting points

            if(previous_path_size < 1)
            {
              previous_path_end_s = car_s;
              previous_path_end_d = car_d;
            }

            // set current position
            VectorXd start_position(2);
            VectorXd next_position(2);

            start_position << previous_path_end_s,
                              previous_path_end_d;
            next_position << 0,0;

            //debug
            cout << "--------------- start ------------------------"<<endl;
            cout << "previous_path_size = " << previous_path_size << endl;
            cout << "car_s = " << car_s << endl;
            cout << "car_d = " << car_d << endl;
            cout << "car_x = " << car_x << endl;
            cout << "car_y = " << car_y << endl;
            cout << "previous_path_end_s = " << previous_path_end_s << endl;
            cout << "previous_path_end_d = " << previous_path_end_d << endl;
            cout << endl << endl;
            //end debug
            int n_rest_points = N_PATH_POINTS - previous_path_size;
            for(int i=0; i < n_rest_points; i++)
            {

              /* compute next position */
              // get velocity in frenet
              // use this , const double epsi = 1e-4;

              // Potential_field pf;
              // potential_func = pf.potential_function(goal,obstacles);
              // VectorXd grad = pf.gradient(start_position , potential_func);
              VectorXd velocity(2);
              velocity << 0.224*5 , 0;

              // double speed = velocity.norm();
              //
              // // check maximum speed
              // if(speed > MAX_SPEED)
              // {
              //   velocity = MAX_SPEED * velocity.normalized();
              //   // add , limiting MAX_ACCEL
              // }

              // compute next position
              // next_position = start_position + (DELTA_T * velocity);
              VectorXd add(2);
              add << 5 , 0;
              next_position = start_position + add;

              // change Frenet to Cartesian
              double next_position_s = next_position(0);
              double next_position_d = next_position(1);

              vector<double> next_position_xy(2);
              next_position_xy = getXY(next_position_s,
                                      next_position_d,
                                      map_waypoints_s,
                                      map_waypoints_x,
                                      map_waypoints_y);

              // save postion
              new_path_x.push_back(next_position_xy[0]);
              new_path_y.push_back(next_position_xy[1]);

              // set current position
              start_position = next_position;
            }

            //debug
            cout << "new_path_x[new_path_x.size()-1] = "<< new_path_x[new_path_x.size()-1] << endl;
            cout << "new_path_y[new_path_y.size()-1] = "<< new_path_y[new_path_y.size()-1] << endl;
            cout << endl;
            //end debug


            // save next_path
            // next_path_x,y = previous_path_x,y + new_path_x,y
            // if(previous_path_size < 1)
            // {
            //   next_path_x.push_back(car_x);
            //   next_path_y.push_back(car_y);
            // }
            // else
            // {
            //   for(int i=0; i < previous_path_size; i++)
            //   {
            //     next_path_x.push_back(previous_path_x[i]);
            //     next_path_y.push_back(previous_path_y[i]);
            //   }
            // }
            //
            //
            // for(int i=0; i < new_path_x.size(); i++)
            // {
            //   next_path_x.push_back(new_path_x[i]);
            //   next_path_y.push_back(new_path_y[i]);
            // }
            //


            // double dist_inc = 0.3;
            // for(int i = 0; i < 30; i++)
            // {
            //       double next_s = car_s + dist_inc * i;
            //       double next_d = car_d;
            //
            //       vector<double> next_xy(2);
            //       next_xy = getXY(next_s,
            //                       next_d,
            //                       map_waypoints_s,
            //                       map_waypoints_x,
            //                       map_waypoints_y);
            //
            //       double next_x = next_xy[0] ;
            //       double next_y = next_xy[1];
            //
            //       next_path_x.push_back(next_x);
            //       next_path_y.push_back(next_y);
            // }

            double dist_inc = 0.5;
           for(int i = 0; i < 50; i++)
           {
                 next_path_x.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                 next_path_y.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
           }

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
  // program,  doesn't compile :-(
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
