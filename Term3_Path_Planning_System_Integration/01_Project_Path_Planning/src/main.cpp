#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
// #include "Eigen-3.3/Eigen/Dense"

#include "json.hpp"
// #include "spline.h"
// #include "helper.h"
#include "path_planner.h"

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

            // ego car state
            vector<double> car_state;
            car_state.push_back(car_x);
            car_state.push_back(car_y);
            car_state.push_back(car_s);
            car_state.push_back(car_d);
            car_state.push_back(car_yaw);
            car_state.push_back(car_speed);

            // map waypoints
            // vector<vector<double>> map_waypoints;
            // map_waypoints.push_back(map_waypoints_x);
            // map_waypoints.push_back(map_waypoints_y);
            // map_waypoints.push_back(map_waypoints_s);
            // map_waypoints.push_back(map_waypoints_dx);
            // map_waypoints.push_back(map_waypoints_dy);

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

            vector<vector<double>> previous_path_xy;
            previous_path_xy.push_back(previous_path_x);
            previous_path_xy.push_back(previous_path_y);

            //debug
            cout << "--------------- in main ------------------------"<<"\n";
            cout << "previous_path_xy[0].size() = " << previous_path_xy[0].size() <<"\n";
            cout << "previous_path_xy.size() = " << previous_path_xy.size() <<"\n";
            //debug

            vector<vector <double>> new_pts_xy(2);
            vector<vector <double>> new_pts_sd(2);

            // call path planner
            new_pts_sd = get_new_path(car_state,
                                      sensor_fusion,
                                      previous_path_xy,
                                      map_waypoints_x,
                                      map_waypoints_y);

            // convert coordinate and save new waypoints
            int new_pts_size = new_pts_sd[0].size();
            for(int i = 0; i < new_pts_size; i++)
            {
                  vector<double> next_xy(2);
                  next_xy = getXY(new_pts_sd[0][i], // frenet s
                                  new_pts_sd[1][i], // frenet d
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y);

                  new_pts_xy[0].push_back(next_xy[0]);
                  new_pts_xy[1].push_back(next_xy[1]);
            }

            for(int i=0; i<previous_path_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

              // //debug
              // cout << "\n";
              // cout << "previous_path_x,y = " << previous_path_x[i]<< previous_path_y[i] <<"\n";
              // cout << "\n";
              // //end debug
            }

            for(int i=0; i<new_pts_size; i++)
            {
              next_x_vals.push_back(new_pts_xy[0][i]);
              next_y_vals.push_back(new_pts_xy[1][i]);

              // //debug
              // cout << "\n";
              // cout << "new_pts_xy = " << new_pts_xy[0][i]<<new_pts_xy[1][i] <<"\n";
              // cout << "\n";
              // //end debug
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
