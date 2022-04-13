#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <string>
#include <vector>

using namespace std;

enum Lanes
{
    LEFT_LANE,
    CENTER_LANE,
    RIGHT_LANE,
};

class Vehicle {
public:
  // Constructors
  Vehicle();
  Vehicle(int lane, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y,
          string state="CS");
  std::pair<vector<double>, vector<double>> generate_trajectory(double ref_vel, int lane);
  void setPreviousPath(vector<double> previous_path_x, vector<double>previous_path_y, double end_path_s);
  void updateLocalization(double car_x, double car_y, double car_s, double car_yaw);


private:
    vector<string> successor_states();

    int current_lane_, s_;
    float v_, a_, max_acceleration_;
    string state_;
    string current_state_;
    std::vector<double> previous_path_x_, previous_path_y_;
    double car_x_, car_y_, car_s_, car_yaw_;
    int prev_size_;
    double end_path_s_;
    vector<double> map_waypoints_s_, map_waypoints_x_, map_waypoints_y_;
};


#endif //PATH_PLANNING_VEHICLE_H
