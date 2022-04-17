#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <string>
#include <vector>

using namespace std;

class Vehicle {
public:
  // Constructors
  Vehicle();
  Vehicle(int lane, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y,
          double max_speed_change, double SPEED_LIMIT, string state="CS");
  std::pair<vector<double>, vector<double>> generate_trajectory(double ref_vel, int lane);
  void setPreviousPath(vector<double> previous_path_x, vector<double>previous_path_y, double end_path_s);
  void updateLocalization(double car_x, double car_y, double car_s, double car_v, double car_yaw);
  void updatePredictions(vector<vector<double>> predictions);
  std::pair<vector<double>, vector<double>> choose_next_state();


private:
    vector<string> successor_states();
    std::pair<vector<double>, vector<double>> generate_trajectory(string state);
    std::pair<vector<double>, vector<double>> keep_lane_trajectory();
    bool get_vehicle_ahead(vector<double> &rVehicle, int lane);
    bool get_vehicle_behind(vector<double> &rVehicle, int lane);
    std::pair<vector<double>, vector<double>> get_kinematics(int lane);

    int current_lane_, s_;
    float v_, a_, max_acceleration_;
    double SPEED_LIMIT_;
    string current_state_;
    std::vector<double> previous_path_x_, previous_path_y_;
    double car_x_, car_y_, car_s_, car_v_, car_yaw_;
    double ref_vel_;
    int prev_size_;
    double end_path_s_;
    double max_speed_change_;
    vector<double> map_waypoints_s_, map_waypoints_x_, map_waypoints_y_;
    vector<vector<double>> predictions_;
};


#endif //PATH_PLANNING_VEHICLE_H
