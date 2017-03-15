#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"

#include <dynamic_reconfigure/server.h>
#include <overtaking_maneuver/OvertakingManeuverInputsConfig.h>
#include <string>
#include <cmath>

std::string node_name = "overtaking_maneuver_node";
std::string pub_path = "overtaking_path";

// INPUTS
// V (>= 5 m/s): Initial and final velocity [m/s]
// W: Width of the lane or of the diversion / total y-direction distance [m]
// A: Magnitude of the maximal resultant acceleration of ego vehicle [m/s^2]
double input_vel = 15; // 2.77; // = 10 km/s
double input_width = 3.00;
double input_max_acc = 3.00; // = normal car in the US

// Simple case
double input_vel_obstacle = 0;
// Time step size
double time_step_size = 0.1;

double calculate_total_dis(double input_vel, double input_width,
                           double input_max_acc) {
  return 2.4 * input_vel * sqrt(input_width / input_max_acc);
}

double calculate_total_time(double input_vel, double input_width,
                            double input_max_acc) {
  return sqrt(3) * pow(input_width, 3 / 2) * sqrt(input_max_acc) /
             pow(input_vel, 2) +
         2.4 * sqrt(input_width) / sqrt(input_max_acc);
}

double calculate_x_at_t(double input_vel, double dis, double total_time,
                        double time_t) {
  return input_vel * time_t +
         (input_vel * total_time - dis) * (-10 * pow((time_t / total_time), 3) +
                                           15 * pow((time_t / total_time), 4) -
                                           6 * pow((time_t / total_time), 5));
}

double calculate_y_at_t(double input_width, double total_time, double time_t) {
  return input_width +
         input_width * (-10 * pow((time_t / total_time), 3) +
                        15 * pow((time_t / total_time), 4) -
                        6 * pow((time_t / total_time), 5));
}

void dynamic_config_callback(
    overtaking_maneuver::OvertakingManeuverInputsConfig &config,
    uint32_t level) {
  input_vel = config.input_vel;
  input_width = config.input_width;
  input_max_acc = config.input_max_acc;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, node_name);

  ros::NodeHandle n;
  ros::Publisher pub_trajectory = n.advertise<nav_msgs::Path>(pub_path, 1000);
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<
      overtaking_maneuver::OvertakingManeuverInputsConfig> server;
  dynamic_reconfigure::Server<
      overtaking_maneuver::OvertakingManeuverInputsConfig>::CallbackType f;
  f = boost::bind(&dynamic_config_callback, _1, _2);
  server.setCallback(f);

  while (ros::ok()) {

    // OUTPUTS
    // Trajectory: x(t) and y(t)
    // D: Total x-direction distance
    // T: Total time for maneuver

    double d = calculate_total_dis(input_vel, input_width, input_max_acc);
    double t = calculate_total_time(input_vel, input_width, input_max_acc);
    ROS_INFO("vel %f, width %f, max_acc %f", input_vel, input_width,
             input_max_acc);
    ROS_INFO("d %f", d);
    ROS_INFO("t %f", t);
    double x_start = calculate_x_at_t(input_vel, d, t, 0);
    double y_start = calculate_y_at_t(input_width, t, 0);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
