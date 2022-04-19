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
                 double max_speed_change, double SPEED_LIMIT, std::string state) {
  current_lane_ = lane;
  current_state_ = state;
  max_acceleration_ = -1;
  map_waypoints_s_ = map_waypoints_s;
  map_waypoints_x_ = map_waypoints_x;
  map_waypoints_y_ = map_waypoints_y;
  max_speed_change_ = max_speed_change;
  SPEED_LIMIT_ = SPEED_LIMIT;
  ref_vel_ = 0.0;
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

void Vehicle::updateLocalization(double car_x, double car_y, double car_s, double car_v, double car_yaw)
{
  car_x_ = car_x;
  car_y_ = car_y;
  car_s_ = car_s;
  car_yaw_ = car_yaw;
  car_v_ = car_v;

  if(prev_size_>0)
  {
    car_s_ = end_path_s_;
  }
}


void Vehicle::setPreviousPath(vector<double> previous_path_x, vector<double>previous_path_y, double end_path_s)
{
  previous_path_x_ = std::move(previous_path_x);
  previous_path_y_ = std::move(previous_path_y);

  prev_size_ = previous_path_x_.size();
  end_path_s_ = end_path_s;
}


void Vehicle::updatePredictions(vector<vector<double>> predictions)
{
  predictions_ = predictions;
}


bool Vehicle::get_vehicle_behind(vector<double> &rVehicle, int lane) {
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;

  for (auto vehicle:predictions_)
  {
    auto d = static_cast<float>(vehicle[6]);
    double vx = vehicle[3];
    double vy = vehicle[4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = vehicle[5];

    check_car_s += ((double)prev_size_ * .02 * check_speed); // if using previous points can project s value
    //out check s values greater than mine and s gap

    if (isSameLane(d, lane) && check_car_s < car_s_ && check_car_s > max_s && check_car_s > car_s_ - 30) {
      max_s = check_car_s;
      rVehicle = vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}


bool Vehicle::get_vehicle_ahead(vector<double> &rVehicle, int lane) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double min_s = std::numeric_limits<double>::max();
  bool found_vehicle = false;

  for(auto vehicle:predictions_){
    auto d = static_cast<float>(vehicle[6]);
    double vx = vehicle[3];
    double vy = vehicle[4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = vehicle[5];

    check_car_s += ((double)prev_size_ * .02 * check_speed); // if using previous points can project s value
                                                             //out check s values greater than mine and s gap

    if (isSameLane(d, lane) && check_car_s > car_s_ && check_car_s < min_s && (check_car_s - car_s_) < 30) {
      min_s = check_car_s;
      rVehicle = vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}


double Vehicle::get_kinematics(int lane)
{
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
//  double max_velocity_accel_limit = max_speed_change_ + car_v_;
  double max_velocity_accel_limit = ref_vel_ + max_speed_change_;
  double new_velocity;

  vector<double> vehicle_ahead;
  vector<double> vehicle_behind;

  if (get_vehicle_ahead(vehicle_ahead, lane)) {
    if (get_vehicle_behind(vehicle_behind, lane)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      double vx = vehicle_ahead[3];
      double vy = vehicle_ahead[4];
      double vehicle_ahead_speed = sqrt(vx*vx+vy*vy);
      double vehicle_ahead_speed_mph = vehicle_ahead_speed * 2.224;
      double speed_change = (ref_vel_ - vehicle_ahead_speed_mph > max_speed_change_)  ? max_speed_change_ : ref_vel_ - vehicle_ahead_speed_mph;
      new_velocity = std::max((ref_vel_ - speed_change), vehicle_ahead_speed_mph);

    } else {
      double vx = vehicle_ahead[3];
      double vy = vehicle_ahead[4];
      double vehicle_ahead_speed = sqrt(vx*vx+vy*vy);
      double vehicle_ahead_s = vehicle_ahead[5];
      vehicle_ahead_s += ((double)prev_size_ * .02 * vehicle_ahead_speed);

      double vehicle_ahead_speed_mph = vehicle_ahead_speed * 2.224;
//      double max_velocity_in_front = (vehicle_ahead_s - car_s_ - this->preferred_buffer) + vehicle_ahead.v
//                                    - 0.5 * (this->a);
//      double max_velocity_in_front = vehicle_ahead_speed - max_speed_change_;
      double speed_change = (ref_vel_ - vehicle_ahead_speed_mph > max_speed_change_)  ? max_speed_change_ : ref_vel_ - vehicle_ahead_speed_mph;
      double max_velocity_in_front = std::max((ref_vel_ - speed_change), vehicle_ahead_speed_mph);

      new_velocity = std::min(std::min(max_velocity_in_front,
                                       max_velocity_accel_limit),
                              SPEED_LIMIT_);
    }
  } else {
    new_velocity = std::min(max_velocity_accel_limit, SPEED_LIMIT_);
  }
  return new_velocity;
}

ego_state Vehicle::keep_lane_trajectory()
{
  // Generate a keep lane trajectory.
  double new_velocty = get_kinematics(current_lane_);

  ego_state new_state;
  new_state.lane = current_lane_;
  new_state.velocity = new_velocty;
  new_state.state = "KL";

  return new_state;
}

ego_state Vehicle::prep_lane_change_trajectory(string state)
{
  // Generate a trajectory preparing for a lane change.
  ego_state next_state;
  double new_v = 0;
  vector<double> vehicle_behind;

  int new_lane = current_lane_ + lane_direction_[state];
  if(new_lane < 0 || new_lane > 2)
    return next_state;

  double curr_lane_new_velocity = get_kinematics(current_lane_);

  if (get_vehicle_behind(vehicle_behind, current_lane_)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_v = curr_lane_new_velocity;
  } else {
    double best_velocity;
    double next_lane_velocity = get_kinematics(new_lane);
    // Choose kinematics with lowest velocity.
    if (next_lane_velocity < curr_lane_new_velocity) {
      best_velocity = next_lane_velocity;
    } else {
      best_velocity = curr_lane_new_velocity;
    }
    new_v = best_velocity;
  }
  next_state.velocity = new_v;
  next_state.lane = current_lane_;
  next_state.state = state;

  return next_state;
}

ego_state Vehicle::lane_change_trajectory(string state) {
  // Generate a lane change trajectory.
  int new_lane = current_lane_ + lane_direction_[state];
  ego_state next_state;

  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for(auto vehicle:predictions_){
    auto d = static_cast<float>(vehicle[6]);
    double vx = vehicle[3];
    double vy = vehicle[4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = vehicle[5];

    check_car_s += ((double)prev_size_ * .02 * check_speed); // if using previous points can project s value
    //out check s values greater than mine and s gap

    if (isSameLane(d, new_lane) && check_car_s > car_s_ - 10 && check_car_s < car_s_ + 30) {
      // If lane change is not possible, return empty trajectory.
      return next_state;
    }
  }
  double next_lane_velocity = get_kinematics(new_lane);

  next_state.velocity = next_lane_velocity;
  next_state.state = state;
  next_state.lane = new_lane;

  return next_state;
}


ego_state Vehicle::generate_trajectory(string state)
{
  std::pair<vector<double>, vector<double>> trajectory;
  ego_state new_state;

  if (state == "KL"){
    new_state = keep_lane_trajectory();
  }
  else if (state == "LCL" || state == "LCR") {
    new_state = lane_change_trajectory(state);
  }
  else if (state == "PLCL" || state == "PLCR") {
    new_state = prep_lane_change_trajectory(state);
  }

  return new_state;
}

std::pair<vector<double>, vector<double>> Vehicle::choose_next_state()
{
  std::pair<vector<double>, vector<double>> final_trajectory;
  ego_state next_state;
  vector<ego_state> final_state;
  vector<float> costs;

  vector<string> states = successor_states();

  for(vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    ego_state state = generate_trajectory(*it);
    if (state.velocity > 0.0001)
    {
      vector<double> vehicle_ahead;
      int intended_lane;
      int final_lane;

      if (state.state == "PLCL") {
        intended_lane = state.lane - 1;
        final_lane = state.lane;
      } else if (state.state == "PLCR") {
        intended_lane = state.lane + 1;
        final_lane = state.lane;
      } else {
        intended_lane = state.lane;
        final_lane = state.lane;
      }

      double proposed_speed_intended = SPEED_LIMIT_;

      if (get_vehicle_ahead(vehicle_ahead, intended_lane))
      {
        double vx = vehicle_ahead[3];
        double vy = vehicle_ahead[4];
        double vehicle_ahead_speed = sqrt(vx*vx+vy*vy);
        double vehicle_ahead_speed_mph = vehicle_ahead_speed * 2.224;

        proposed_speed_intended = vehicle_ahead_speed_mph;
      }

      double proposed_speed_final = SPEED_LIMIT_;

      if (get_vehicle_ahead(vehicle_ahead, final_lane))
      {
        double vx = vehicle_ahead[3];
        double vy = vehicle_ahead[4];
        double vehicle_ahead_speed = sqrt(vx*vx+vy*vy);
        double vehicle_ahead_speed_mph = vehicle_ahead_speed * 2.224;

        proposed_speed_final = vehicle_ahead_speed_mph;
      }



      float cost = (2.0 * SPEED_LIMIT_ - proposed_speed_intended - proposed_speed_final)/SPEED_LIMIT_;
      costs.emplace_back(cost);
      final_state.emplace_back(state);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  next_state = final_state[best_idx];

  ref_vel_ = next_state.velocity;
  current_state_ = next_state.state;
  current_lane_ = next_state.lane;
//  std::cout << current_state_ << " " << current_lane_ << " " << ref_vel_ << std::endl;

  return generate_trajectory(next_state.velocity, next_state.lane);
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
  double SPEED_LIMIT = 49.5; //mph

  // At each timestep, ego can set acceleration to value between
  //   -MAX_ACCEL and MAX_ACCEL
  int MAX_ACCEL = 10; // m/s^2

  double max_speed_change = MAX_ACCEL * .02 * 2.224;

  string current_state = "KL";

  Vehicle vehicle(lane, map_waypoints_s, map_waypoints_x, map_waypoints_y,
                  max_speed_change, SPEED_LIMIT, current_state);

  h.onMessage([&lane, &ref_vel, &max_speed_change, &vehicle]
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

          vehicle.setPreviousPath(previous_path_x, previous_path_y, end_path_s);
          vehicle.updateLocalization(car_x, car_y, car_s, car_speed, car_yaw);
          vehicle.updatePredictions(sensor_fusion);

          //Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          std::tie(next_x_vals,next_y_vals) = vehicle.choose_next_state();

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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