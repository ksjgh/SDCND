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

            // map waypoints, data pack of waypoints
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
          	const double previous_path_end_s = j[1]["end_path_s"];
          	const double previous_path_end_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	// auto sensor_fusion = j[1]["sensor_fusion"];
            const vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            const int previous_path_size = previous_path_x.size();

            // //debug
            // cout << "--------------- in main ------------------------"<<"\n";
            // cout << "previous_path_xy[0].size() = " << previous_path_xy[0].size() <<"\n";
            // // cout << "previous_path_xy.size() = " << previous_path_xy.size() <<"\n";
            // //debug

            vector<vector<double>> new_pts_xy(2);
            vector<vector<double>> new_pts_sd(2);

            /* if previous size is almost empty, use current position
               as starting reference */
            // this is required for calulating smoothing path
            double ref_x, ref_y, ref_yaw;
            if(previous_path_size < 2)
            {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ref_x = car_x;
              ref_y = car_y;
              ref_yaw = car_yaw;

              new_pts_xy[0].push_back(prev_car_x);
              new_pts_xy[0].push_back(car_x);

              new_pts_xy[1].push_back(prev_car_y);
              new_pts_xy[1].push_back(car_y);
            }
            // use the previous path's end point as starting reference
            // to get smooth trasition
            else
            {
              // redefine reference state as previous path end point
              ref_x = previous_path_x[previous_path_size-1];
              ref_y = previous_path_y[previous_path_size-1];

              double ref_x_prev = previous_path_x[previous_path_size-2];
              double ref_y_prev = previous_path_y[previous_path_size-2];
              ref_yaw = atan2( ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use tow points that make the path tangent to the previous path's end points
              new_pts_xy[0].push_back(ref_x_prev);
              new_pts_xy[0].push_back(ref_x);

              new_pts_xy[1].push_back(ref_y_prev);
              new_pts_xy[1].push_back(ref_y);
            }

            // call path planner
            new_pts_sd = get_new_path(car_state,
                                      sensor_fusion,
                                      previous_path_size,
                                      previous_path_end_s,
                                      previous_path_end_d,
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

                // //debug
                // cout <<endl;
                // cout << "new_pts_s,d = " << new_pts_sd[0][i] <<" , "<<new_pts_sd[1][i] <<endl;
                // cout << endl;
                // // //end debug

                // //debug
                // if(i>0)
                // {
                //   if(new_pts_sd[0][i] < new_pts_sd[0][i-1])
                //   {
                //     cout << endl;
                //     cout << "s_prev = " << new_pts_sd[0][i-1] << endl;
                //     cout << "s_now  = " << new_pts_sd[0][i]   << endl;
                //     cout << endl;
                //   }
                // }
                // //debug end
            }

            // convert global coordinates into car's local coordinates for easy math

            for(int i=0; i < new_pts_xy[0].size(); i++)
            {
              double global_x =  new_pts_xy[0][i];
              double global_y =  new_pts_xy[1][i];

              double shift_x = global_x - ref_x;
              double shift_y = global_y - ref_y;

              double local_x = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
              double local_y = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);

              new_pts_xy[0][i] = local_x;
              new_pts_xy[1][i] = local_y;

            }

            // smoothing path using polynomial

            double* ptrx = &new_pts_xy[0][0];
            Eigen::Map<Eigen::VectorXd> new_pts_x(ptrx, new_pts_xy[0].size());

            double* ptry = &new_pts_xy[1][0];
            Eigen::Map<Eigen::VectorXd> new_pts_y(ptry, new_pts_xy[0].size());

            auto coeff = polyfit(new_pts_x , new_pts_y ,3);

            // //debug
            // cout << "\n";
            // cout << "previous_path_x,y = " << previous_path_x[i]<< previous_path_y[i] <<"\n";
            // cout << "\n";
            // //end debug


            // calculate how to break up spline points so that we travel at our desired reference velocity
            double end_x = new_pts_xy[0].back();
            double end_y = polyeval(coeff,end_x);
            double begin_x = new_pts_xy[0][0];
            double begin_y = polyeval(coeff,begin_x);
            double start_x = new_pts_xy[0][1];
            double start_y = polyeval(coeff,start_x);
            double target_dist = distance(end_x,end_y,begin_x,begin_y);
            double ave_vel = target_dist/(DELTA_T * new_pts_xy[0].size());
            double x_delta = (end_x - 0)/(double)new_pts_size;
            double x_prev = start_x;

            // get new points from smoothed spline
            int n_rest_points = N_PATH_POINTS - previous_path_size;//check can be replace with new_pts_size
            new_pts_xy[0].clear();
            new_pts_xy[1].clear();

            for(int i=0; i < new_pts_size; i++)
            {
              // get each (x,y) point in spline
              double x_point = x_prev + x_delta;
              double y_point = polyeval(coeff,x_point);
              x_prev = x_point;

              //rotate & shift , for back to global coordinates
              x_point = (x_point*cos(ref_yaw) - y_point*sin(ref_yaw));
              y_point = (x_point*sin(ref_yaw) + y_point*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              // append ponit to trajectory
              new_pts_xy[0].push_back(x_point);
              new_pts_xy[1].push_back(y_point);
            }

            // add previous path
            // next_x,y_vals = previous_path_x,y + new_pts_x,y
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

            // add new path
            // next_x,y_vals = previous_path_x,y + new_pts_x,y
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
