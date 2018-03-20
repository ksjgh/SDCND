#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

// define sensor fusion data access index
// 0) car's ID,
// 1) car's x position in map coordinates,
// 2) car's y position in map coordinates,
// 3) car's x velocity in m/s,
// 4) car's y velocity in m/s,
// 5) car's s position in frenet coordinates,
// 6) car's d position in frenet coordinates
#define SF_IDX_ID  0
#define SF_IDX_X   1
#define SF_IDX_Y   2
#define SF_IDX_VX  3 // cf.) double vx = sensor_fusion[i][SF_IDX_VX]
#define SF_IDX_VY  4 // cf.) double vy = sensor_fusion[i][SF_IDX_VY]
#define SF_IDX_S   5
#define SF_IDX_D   6

// total number of points in the path planner, 20ms between points
// N_PATH_POINTS * 20ms = behavior horizon , ex) 50 * 20ms = 1.0[s]
#define N_PATH_POINTS (50)

#define TIME_INTERVAL (0.02) // time interval between points 20ms
#define MPS_TO_MPH (0.2236936) // 1 m/s = 0.2236936 mph
#define MPH_TO_MPS (0.44704) // 1 mph = 0.44704 m/s
#define SPEED_STEP (1.0*MPS_TO_MPH) // [m/s]
#define MAX_SPEED (49.5) // speed limit [mph]
#define SAFE_DISTANCE (30.0) // safe distance to front/rear car
#define SPLINE_TARGET_X (30.0) // break up x-distance of spline


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y,
  vector<double> maps_dx, vector<double> maps_dy)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

  double hx = map_x - x;
  double hy = map_y - y;

  double nx = maps_dx[closestWaypoint];
  double ny = maps_dy[closestWaypoint];

  double vx = -ny;
  double vy = nx;

  double inner = hx*vx + hy*vy;

  if (inner < 0.0) {
    closestWaypoint++;
  }

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y,
  vector<double> maps_dx, vector<double> maps_dy)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y, maps_dx, maps_dy);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),(maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s * cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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
    // x and y are the waypoint's map coordinate position,
    // the s value is the distance along the road to get to that waypoint in meters(from 0 to 6945.554)
    // The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road.
    // The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint
    // just add the waypoint's (x,y) coordinates with the d vector multiplied by 2.
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

  // start at lane 1
  int lane = 1;

  // reference velocity of my car
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane]
  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

        	// my car's localization Data
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
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

        	// Sensor Fusion Data, a list of all other cars on the same side of the road: [id, x, y, vx, vy, s, d]
        	vector<vector<double> > sensor_fusion = j[1]["sensor_fusion"];

          int previous_path_size = previous_path_x.size();

          bool speed_down = false;
          bool left_lane_empty = true;
          bool right_lane_empty = true;

          // predict my car's position at behavior horizon
          if (previous_path_size > 0)
          {
            car_s = end_path_s;
          }

          // check other car's current state, predict state and decide my car's action
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // get other car's state : [ id, x, y, vx, vy, s, d]
            // oc_ , means 'other car'
            float  oc_d = sensor_fusion[i][SF_IDX_D];
            double oc_vx = sensor_fusion[i][SF_IDX_VX];
            double oc_vy = sensor_fusion[i][SF_IDX_VY];
            double oc_speed = sqrt(oc_vx*oc_vx + oc_vy*oc_vy);
            double oc_s = sensor_fusion[i][SF_IDX_S];

            // predict other car's position at behavior horizon
            // assumption : only moves in 's'- direction
            oc_s += ((double)previous_path_size * (TIME_INTERVAL) * oc_speed);

            // predict other car's state at behavior horizon
            bool at_same_lane = oc_d > (2 + 4 * lane - 2) && oc_d < (2 + 4 * lane + 2);
            bool at_left_lane = oc_d > (2 + 4 * (lane - 1) - 2) &&  oc_d < (2 + 4 * (lane - 1) + 2);
            bool at_right_lane = oc_d > (2 + 4 * (lane + 1) - 2) && oc_d < (2 + 4 * (lane + 1) + 2);
            bool too_close = abs(oc_s - car_s) < SAFE_DISTANCE;
            bool at_front = oc_s > car_s;

            // other car is too close
            if (at_same_lane && too_close && at_front)
            {
              speed_down = true;
            }

            // check if the left lane is safe for lane change
            if (at_left_lane && too_close)
            {
              left_lane_empty = false;
            }

            // check if the right lane is safe for lane change
            if (at_right_lane && too_close)
            {
              right_lane_empty = false;
            }
          }

          // Decelerate to keep safe distance
          if (speed_down)
          {
            ref_vel -= SPEED_STEP;

            //change to left lane
            if (lane > 0 && left_lane_empty)
            {
              lane--;
            }
            //change to right lane
            else if (lane < 2 && right_lane_empty)
            {
              lane++;
            }
          }
          // Accelerate if no other car is within safe distance
          else if (ref_vel < MAX_SPEED)
          {
            ref_vel += SPEED_STEP;
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interoplate these waypoints with a spline and fill it in with more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw states
          // either we will reference the starting point as where the car is or at the previouss paths end point

          // if previous size is almost empty, use the car as starting reference
          double ref_x_prev, ref_y_prev, ref_x, ref_y, ref_yaw;

          if (previous_path_size < 2) {
            ref_x_prev = car_x - cos(car_yaw);
            ref_y_prev = car_y - sin(car_yaw);
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
          }
          // use the previous path's end point as starting reference to get smooth trasition
          else {
            ref_x_prev = previous_path_x[previous_path_size-2];
            ref_y_prev = previous_path_y[previous_path_size-2];
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // Use two points that make the path tangent to the previous path's end point
          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);

          // In Frenet add evenly 30m spaced points ahead of the starting reference
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
          for (int i = 0; i < ptsx.size(); i++ ) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = (double)SPLINE_TARGET_X;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double N = (target_dist / (TIME_INTERVAL * ref_vel * MPH_TO_MPS));
          // double x_add_on = 0;
          double x_delta = target_x/N;
          double x_prev = 0;

          // Fill up the rest of our path planner points
          for (int i = 1; i <= N_PATH_POINTS - previous_path_x.size(); i++)
          {
            // double x_point = x_add_on + (target_x) / N;
            // double y_point = s(x_point);
            //
            // x_add_on = x_point;

            // get each (x,y) point in spline
            double x_point = x_prev + x_delta;
            double y_point = s(x_point);

            x_prev = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

        	json msgJson;
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
