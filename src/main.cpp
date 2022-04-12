#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle()= default;

Vehicle::Vehicle(int lane,vector<double> map_waypoints_s,vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                 std::string state) {
  current_lane_ = lane;
  state_ = state;
  max_acceleration_ = -1;
  map_waypoints_s_ = map_waypoints_s;
  map_waypoints_x_ = map_waypoints_x;
  map_waypoints_y_ = map_waypoints_y;
}

std::vector<std::string> Vehicle::successor_states() {
  vector<string> states;
  states.emplace_back("KL");

  if(current_state_ == "KL")
  {
    states.emplace_back("PLCL");
    states.emplace_back("PLCR");
  }
  else if (current_state_ == "PLCL")
  {
    if (current_lane_ != LEFT_LANE)
    {
      states.emplace_back("PLCL");
      states.emplace_back("LCL");
    }
  }
  else if (current_state_ == "PLCR")
  {
    if (current_lane_ != RIGHT_LANE)
    {
      states.emplace_back("PLCR");
      states.emplace_back("LCR");
    }
  }

  return states;
}


std::pair<vector<double>, vector<double>> Vehicle::generate_trajectory(double ref_vel, int lane) {

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // later we will interpolate these waypoints with a spline and fill it in with more points that control spaced
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states
  // either we will reference the starting point as where the car is or at the previous paths end point
  double ref_x = car_x_;
  double ref_y = car_y_;
  double ref_yaw = deg2rad(car_yaw_);

  //if previous size is almost empty, use the car as starting reference
  if(prev_size_ < 2)
  {
    //Use two points that makes the path tangent to the car
    double prev_car_x = car_x_ - cos(car_yaw_);
    double prev_car_y = car_y_ - sin(car_yaw_);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x_);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y_);
  }
    //use the previous path's end point as starting reference
  else
  {
    //Redefine reference state as previous path end point
    ref_x = previous_path_x_[prev_size_-1];
    ref_y = previous_path_y_[prev_size_-1];

    double ref_x_prev = previous_path_x_[prev_size_-2];
    double ref_y_prev = previous_path_y_[prev_size_-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    //Use two points that makes the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }


  //In Frenet add evenly 30m spaced points ahead of the starting reference
//  vector<double> next_wp0 = getXY(car_s_+30, (2+4*current_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
//  vector<double> next_wp1 = getXY(car_s_+60, (2+4*current_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
//  vector<double> next_wp2 = getXY(car_s_+90, (2+4*current_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp0 = getXY(car_s_+30, (2+4*lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp1 = getXY(car_s_+60, (2+4*lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp2 = getXY(car_s_+90, (2+4*lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  for(int i = 0; i < ptsx.size(); i++)
  {
    //shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  //create a spline
  tk::spline s;

  //set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  //Define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //Start with all of the previous path points from last time
  for(int i=0; i < previous_path_x_.size(); i++)
  {
    next_x_vals.push_back(previous_path_x_[i]);
    next_y_vals.push_back(previous_path_y_[i]);
  }

  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for(int i = 0; i <= 50 - previous_path_x_.size(); i++)
  {
    double N = (target_dist/(.02*ref_vel/2.24));
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return std::pair<vector<double>, vector<double>>{next_x_vals, next_y_vals};
}

void Vehicle::update(vector<double> previous_path_x, vector<double>previous_path_y, double car_x, double car_y,
                     double car_s, double car_yaw, double end_path_s)
{
//  previous_path_x_ = std::move(previous_path_x);
//  previous_path_y_ = std::move(previous_path_y);

  previous_path_x_ = previous_path_x;
  previous_path_y_ = previous_path_y;

  prev_size_ = previous_path_x_.size();

  car_x_ = car_x;
  car_y_ = car_y;
  car_s_ = car_s;
  car_yaw_ = car_yaw;

  if(prev_size_>0)
  {
    car_s_ = end_path_s;
  }

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  //start in lane
  int lane = 1;

  //Have a reference velocity to target
  double ref_vel = 0.0; //mph

  // impacts default behavior for most states
  double SPEED_LIMIT = 49.5;

  // At each timestep, ego can set acceleration to value between
  //   -MAX_ACCEL and MAX_ACCEL
  int MAX_ACCEL = 10; // m/s^2

  double max_speed_change = MAX_ACCEL * .02 * 2.224;

  string current_state = "KL";

  Vehicle vehicle(lane, map_waypoints_s, map_waypoints_x, map_waypoints_y,
                  current_state);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &max_speed_change, &vehicle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vehicle.update(previous_path_x, previous_path_y, car_x, car_y, car_s, car_yaw, end_path_s);

          int prev_size = previous_path_x.size();

          if(prev_size>0)
          {
            car_s = end_path_s;
          }


          bool too_close = false;
          bool center_lane_free = true;
          bool left_lane_free = true;
          bool right_lane_free = true;
          double left_speed, center_speed, right_speed;

          vector<bool> possible_lane_change = {true, true, true};
          vector<bool> lane_free = {true, true, true};

          vector<double> costs = {49.5, 49.5, 49.5};
          vector<string> lanes = {"left", "center", "right"};
/*
          vector<string> states = successor_states(current_state, lane);
*/
          /**
          // Debug
          std::cout << "States: ";
          for(const auto& state:states)
          {
            std::cout << state << "  ";
          }
          std::cout << std::endl;
          **/

//          for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
//            vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
//            if (trajectory.size() != 0) {
//              cost = calculate_cost(*this, predictions, trajectory);
//              costs.push_back(cost);
//              final_trajectories.push_back(trajectory);
//            }
//          }

          // find ref_v to use
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed); // if using previous points can project s value
                                                                    //out check s values greater than mine and s gap
            // check if there are other cars on the left lane
            if(d < (2+4*LEFT_LANE+2) && d > (2+4*LEFT_LANE-2))
            {
              if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {
                // Do some logic here, lower reference velocity so we dont crash into the car inform of us, could
                // also flag to try to change lanes
                if(lane == LEFT_LANE)
                {
                  too_close = true;
                }
                left_speed = check_speed;
                costs[0] = left_speed;
                left_lane_free = false;
                lane_free[LEFT_LANE] = false;
              }
              if(lane != LEFT_LANE)
              {
                if((check_car_s < car_s + 10) && (std::abs(check_car_s - car_s) < 10))
                {

                  possible_lane_change[LEFT_LANE] = false;
                }
              }
            }

//            // check if there are other cars on the left of the car
//            if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)) {
//              if (((check_car_s > car_s) && ((check_car_s - car_s) < 30)) || ((check_car_s < car_s) && ((check_car_s - car_s) > -10))) {
//                left_lane_free = false;
//              }
//            }
//            else if (lane == 0)
//            {
//              left_lane_free = false;
//            }

            // check if there are other cars on the left of the car
            if (d < (2 + 4 * CENTER_LANE + 2) && d > (2 + 4 * CENTER_LANE - 2)) {
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                if(lane == CENTER_LANE)
                {
                  too_close = true;
                }
                center_lane_free = false;
                lane_free[CENTER_LANE] = false;
                center_speed = check_speed;
                costs[1] = center_speed;
              }
              if(lane != CENTER_LANE)
              {
                if((check_car_s < car_s + 10) && (std::abs(check_car_s - car_s) < 10))
                {
                  possible_lane_change[CENTER_LANE] = false;
                }
              }
            }
//            else if (lane == 0)
//            {
//              left_lane_free = false;
//            }

//            // check if there are other cars on the right of the car
//            if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)) {
//              if (((check_car_s > car_s) && ((check_car_s - car_s) < 30)) || ((check_car_s < car_s) && ((check_car_s - car_s) > -10))) {
//                right_lane_free = false;
//              }
//            }
//            else if (lane == 2)
//            {
//              right_lane_free = false;
//            }

            // check if there are other cars on the right of the car
            if (d < (2 + 4 * RIGHT_LANE + 2) && d > (2 + 4 * RIGHT_LANE - 2)) {
              if ((check_car_s > car_s ) && ((check_car_s - car_s) < 30)) {
                if(lane == RIGHT_LANE)
                {
                  too_close = true;
                }
                right_lane_free = false;
                lane_free[RIGHT_LANE] = false;
                right_speed = check_speed;
                costs[2] = right_speed;
              }
              if(lane != RIGHT_LANE)
              {
                if((check_car_s < car_s + 10) && (std::abs(check_car_s - car_s) < 10))
                {
                  possible_lane_change[RIGHT_LANE] = false;
                }
              }
            }
//            else if (lane == 2)
//            {
//              right_lane_free = false;
//            }

//            if(left_lane_free && right_lane_free)
//            {
//              right_lane_free = false; // because we are going counterclockwise in a circle
//            }
          }

          vector<double>::iterator best_cost = max_element(begin(costs), end(costs));
          int best_idx = distance(begin(costs), best_cost);

          /**
          for(auto cost:costs)
          {
            std::cout << cost << " , ";
          }
          std::cout << std::endl;
          std::cout << "Costs.size(): " << costs.size() << std::endl;
          std::cout << "best_idx: " << best_idx << std::endl;
          **/

          /**
          std::cout << "possible lane: ";
          for(const auto& lane:possible_lane_change)
          {
            std::cout << lane << "  ";
          }
          std::cout << std::endl;
           **/

          std::cout << "Free lane: ";
          for(const auto& l:lane_free)
          {
            std::cout << l << "  ";
          }
          std::cout << std::endl;

          if(too_close)
          {
            if(lane == CENTER_LANE && !center_lane_free)
            {
              if(left_lane_free && right_lane_free && possible_lane_change[LEFT_LANE])
              {
                lane = LEFT_LANE;
                std::cout << "Change to left lane" << std::endl;
              }else if(!left_lane_free && !right_lane_free)
              {
                if(lane == best_idx)
                {
                  std::cout << "Keep " << lanes[best_idx] << " lane" << std::endl;
                  ref_vel -= max_speed_change;
                }
                else
                {
                  if(possible_lane_change[best_idx])
                  {
                    lane = best_idx;
                    std::cout << "Change to " << lanes[best_idx] << " lane" << std::endl;
                  }
                }
              }else if(left_lane_free && !right_lane_free && possible_lane_change[LEFT_LANE])
              {
                lane = LEFT_LANE;
                std::cout << "Change to left lane" << std::endl;
              }
              else if(!left_lane_free && right_lane_free && possible_lane_change[RIGHT_LANE])
              {
                lane = RIGHT_LANE;
                std::cout << "Change to right lane" << std::endl;
              }
              else if(!possible_lane_change[LEFT_LANE] && !possible_lane_change[RIGHT_LANE])
              {
                lane = CENTER_LANE;
                std::cout << "Keep " << lanes[CENTER_LANE] << " lane" << std::endl;
                ref_vel -= max_speed_change;
              }
            }

            if(lane == LEFT_LANE && !left_lane_free)
            {
              if(center_lane_free && right_lane_free && possible_lane_change[CENTER_LANE])
              {
                lane = CENTER_LANE;
                std::cout << "Change to " << lanes[CENTER_LANE] << " lane" << std::endl;;
              }else if(!center_lane_free && !right_lane_free)
              {
                if(lane == best_idx)
                {
                  std::cout << "Keep " << lanes[best_idx] << " lane" << std::endl;
                  ref_vel -= max_speed_change;
                }
                else
                {
                  if(best_idx == RIGHT_LANE)
                  {
                    if(costs[CENTER_LANE] > costs[LEFT_LANE] && possible_lane_change[CENTER_LANE])
                    {
                      lane = CENTER_LANE;
                      std::cout << "Change to " << lanes[CENTER_LANE] << " lane" << std::endl;
                    }
                    else
                    {
                      lane = LEFT_LANE;
                      std::cout << "Keep " << lanes[LEFT_LANE] << " lane" << std::endl;
                      ref_vel -= max_speed_change;
                    }
                  }
                  else if (best_idx == CENTER_LANE && possible_lane_change[CENTER_LANE])
                  {
                    lane = best_idx;
                    std::cout << "Change to " << lanes[best_idx] << " lane" << std::endl;
                  }
                }
              }else if(center_lane_free && possible_lane_change[CENTER_LANE])
              {
                lane = CENTER_LANE;
                std::cout << "Change to center lane" << std::endl;
              }
              else if(possible_lane_change[CENTER_LANE])
              {
                lane = LEFT_LANE;
                std::cout << "Keep " << lanes[LEFT_LANE] << " lane" << std::endl;
                ref_vel -= max_speed_change;
              }
            }


            if(lane == RIGHT_LANE && !right_lane_free)
            {
              if(left_lane_free && center_lane_free && possible_lane_change[CENTER_LANE])
              {
                lane = CENTER_LANE;
                std::cout << "Change to " << lanes[CENTER_LANE] << " lane" << std::endl;
              }else if(!left_lane_free && !center_lane_free)
              {
                if(lane == best_idx)
                {
                  std::cout << "Keep " << lanes[best_idx] << " lane" << std::endl;
                  ref_vel -= max_speed_change;
                }
                else
                {
                  if(best_idx == LEFT_LANE)
                  {
                    if(costs[CENTER_LANE] > costs[RIGHT_LANE] && possible_lane_change[CENTER_LANE])
                    {
                      lane = CENTER_LANE;
                      std::cout << "Change to " << lanes[CENTER_LANE] << " lane" << std::endl;
                    }
                    else
                    {
                      lane = RIGHT_LANE;
                      std::cout << "Keep " << lanes[RIGHT_LANE] << " lane" << std::endl;
                      ref_vel -= max_speed_change;
                    }
                  }
                  else if (best_idx == CENTER_LANE && possible_lane_change[CENTER_LANE])
                  {
                    lane = best_idx;
                    std::cout << "Change to " << lanes[best_idx] << " lane" << std::endl;
                  }
                }
              }else if(!left_lane_free && center_lane_free && possible_lane_change[CENTER_LANE])
              {
                lane = CENTER_LANE;
                std::cout << "Change to right lane" << std::endl;
              }
              else if(possible_lane_change[CENTER_LANE])
              {
                lane = RIGHT_LANE;
                std::cout << "Keep " << lanes[RIGHT_LANE] << " lane" << std::endl;
                ref_vel -= max_speed_change;
              }
            }


//            if(!left_lane_free && !right_lane_free) // ACC mode
//            {
//              ref_vel -= max_speed_change;
//              std::cout << "ACC Mode" << std::endl;
//            }
//            if (right_lane_free && lane < 2) {
//              lane += 1;
//              std::cout << "Right Lane Free" << std::endl;
//            }
//            if (left_lane_free && lane > 0) {
//              lane -= 1;
//              std::cout << "Left Lane Free" << std::endl;
//
//            }
          }
          else if(ref_vel < 49.5)
          {
            ref_vel += max_speed_change;
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it in with more points that control spaced
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //if previous size is almost empty, use the car as starting reference
          if(prev_size < 2)
          {
            //Use two points that makes the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //use the previous path's end point as starting reference
          else
          {
            //Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            //Use two points that makes the path tangent to the previous path's end point
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

          for(int i = 0; i < ptsx.size(); i++)
          {
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          //create a spline
          tk::spline s;

          //set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          //Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Start with all of the previous path points from last time
          for(int i=0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for(int i = 0; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist/(.02*ref_vel/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          //Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals_2;
          vector<double> next_y_vals_2;

          std::tie(next_x_vals_2,next_y_vals_2) = vehicle.generate_trajectory(ref_vel, lane);

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
//          double dist_inc = 0.3;
//          for (int i = 0; i < 50; ++i) {
//            double next_s = car_s + (i+1) * dist_inc;
//            double next_d = 6;
//            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//
//            next_x_vals.push_back(xy[0]);
//            next_y_vals.push_back(xy[1]);
//          }

//          msgJson["next_x"] = next_x_vals;
//          msgJson["next_y"] = next_y_vals;
          msgJson["next_x"] = next_x_vals_2;
          msgJson["next_y"] = next_y_vals_2;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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