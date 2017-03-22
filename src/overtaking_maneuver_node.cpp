#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <dynamic_reconfigure/server.h>
#include <overtaking_maneuver/OvertakingManeuverInputsConfig.h>
#include <string>
#include <cmath>

std::string node_name = "overtaking_maneuver_node";
std::string pub_path = "overtaking_path";
std::string sub_odom_topic = "odom";
std::string odom_frame_id = "catvehicle/map";

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
double time_step_size = 0.5;
// Current pose of ego vehicle
geometry_msgs::Pose current_pose;

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

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
  current_pose.position.x = odom->pose.pose.position.x;
  current_pose.position.y = odom->pose.pose.position.y;
  current_pose.position.z = odom->pose.pose.position.z;

  current_pose.orientation.x = odom->pose.pose.orientation.x;
  current_pose.orientation.y = odom->pose.pose.orientation.y;
  current_pose.orientation.z = odom->pose.pose.orientation.z;
  current_pose.orientation.w = odom->pose.pose.orientation.w;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, node_name);

  ros::NodeHandle n;
  ros::Publisher pub_trajectory = n.advertise<nav_msgs::Path>(pub_path, 1000);
  ros::Subscriber sub_odom =
      n.subscribe<nav_msgs::Odometry>(sub_odom_topic, 1000, odom_callback);
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<
      overtaking_maneuver::OvertakingManeuverInputsConfig> server;
  dynamic_reconfigure::Server<
      overtaking_maneuver::OvertakingManeuverInputsConfig>::CallbackType f;
  f = boost::bind(&dynamic_config_callback, _1, _2);
  server.setCallback(f);

  while (ros::ok()) {

    // OUTPUTS
    // Trajectory path: x(t) and y(t)
    // D: Total x-direction distance
    // T: Total time for maneuver

    double total_dis =
        calculate_total_dis(input_vel, input_width, input_max_acc);
    double total_time =
        calculate_total_time(input_vel, input_width, input_max_acc);
    ROS_INFO("vel %f, width %f, max_acc %f", input_vel, input_width,
             input_max_acc);
    ROS_INFO("d %f", total_dis);
    ROS_INFO("t %f", total_time);

    // init temp pose
    geometry_msgs::PoseStamped pose_tmp;
    std_msgs::Header pose_tmp_header;
    pose_tmp_header.frame_id = odom_frame_id;
    pose_tmp_header.stamp = ros::Time::now();
    pose_tmp.header = pose_tmp_header;

    // init temp path
    nav_msgs::Path path_tmp;
    std_msgs::Header path_tmp_header;
    path_tmp_header.frame_id = odom_frame_id;
    path_tmp_header.stamp = ros::Time::now();
    path_tmp.header = path_tmp_header;

    double y_t_0 = calculate_y_at_t(input_width, total_time, 0);

    double time = 0;
    while (time <= total_time) {
      double x = calculate_x_at_t(input_vel, total_dis, total_time, time);
      double y = calculate_y_at_t(input_width, total_time, time);

      pose_tmp.pose.position.x = (x - total_dis) + current_pose.position.x;
      pose_tmp.pose.position.y = ((-1) * y + y_t_0) + current_pose.position.y;
      pose_tmp.pose.position.z = (0) + current_pose.position.z;

      pose_tmp.pose.orientation.x = (0) + current_pose.orientation.x;
      pose_tmp.pose.orientation.y = (0) + current_pose.orientation.y;
      pose_tmp.pose.orientation.z = (0) + current_pose.orientation.z;
      pose_tmp.pose.orientation.w = (1) + current_pose.orientation.w;

      path_tmp.poses.push_back(pose_tmp);

      time = time + time_step_size;
    }

    time = 0;
    while (time <= total_time) {
      double x = calculate_x_at_t(input_vel, total_dis, total_time, time);
      double y = calculate_y_at_t(input_width, total_time, time);

      pose_tmp.pose.position.x = (x) + current_pose.position.x;
      pose_tmp.pose.position.y = (y) + current_pose.position.y;
      pose_tmp.pose.position.z = (0) + current_pose.position.z;

      pose_tmp.pose.orientation.x = (0) + current_pose.orientation.x;
      pose_tmp.pose.orientation.y = (0) + current_pose.orientation.y;
      pose_tmp.pose.orientation.z = (0) + current_pose.orientation.z;
      pose_tmp.pose.orientation.w = (1) + current_pose.orientation.w;

      path_tmp.poses.push_back(pose_tmp);

      time = time + time_step_size;
    }

    pub_trajectory.publish(path_tmp);
    path_tmp.poses.clear();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
