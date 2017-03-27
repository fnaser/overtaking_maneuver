#ifndef OVERTAKING_MANEUVER_H
#define OVERTAKING_MANEUVER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <overtaking_maneuver/OvertakingManeuverInputsConfig.h>
#include <string>
#include <cmath>

using namespace std;

class OvertakingManeuver {

private:
  // Time step size
  double time_step_size;
  // Current pose of ego vehicle
  geometry_msgs::PoseStamped current_pose;
  // Params
  bool update_odom, use_dynamic_reconfig;

  // INPUTS
  // V (>= 5 m/s): Initial and final velocity [m/s]
  // W: Width of the lane or of the diversion / total y-direction distance [m]
  // A: Magnitude of the maximal resultant acceleration of ego vehicle [m/s^2]
  double input_vel;
  double input_width;
  double input_max_acc;

  string sub_user_input_topic;
  string sub_odom_topic;
  string pub_path_topic;
  string pub_path_topic_test;
  string pub_current_pose_topic;
  string robot_name;
  string path_frame_id;
  string path_pose_frame_id;

  ros::NodeHandle *n;
  ros::Publisher pub_trajectory, pub_trajectory_test, pub_current_pose;
  ros::Subscriber sub_odom, sub_user_input;

  double calculate_total_dis(double input_vel, double input_width,
                             double input_max_acc);
  double calculate_total_time(double input_vel, double input_width,
                              double input_max_acc);
  double calculate_x_at_t(double input_vel, double dis, double total_time,
                          double time_t);
  double calculate_y_at_t(double input_width, double total_time, double time_t);
  void user_input_callback(const std_msgs::Bool::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void rotate_path(nav_msgs::Path *path, tf::TransformListener *tflistener);

public:
  ~OvertakingManeuver();
  OvertakingManeuver();
  OvertakingManeuver(ros::NodeHandle *n, bool update_odom,
                     bool use_dynamic_reconfig, string sub_user_input_topic,
                     string sub_odom_topic, string pub_path_topic,
                     string pub_path_topic_test, string pub_current_pose_topic,
                     string robot_name, string path_frame_id,
                     string path_pose_frame_id);

  void publish_trajectory(tf::TransformListener *tflistener, double input_vel,
                          double input_width, double input_max_acc);
  void dynamic_config_callback(
      overtaking_maneuver::OvertakingManeuverInputsConfig &config,
      uint32_t level);

  bool get_use_dynamic_reconfig();
  bool get_update_odom();
  void set_update_odom(bool);
};

#endif // OVERTAKING_MANEUVER_H
