#include "overtaking_maneuver/overtaking_maneuver.h"

OvertakingManeuver::~OvertakingManeuver() {}

OvertakingManeuver::OvertakingManeuver() {}

OvertakingManeuver::OvertakingManeuver(
    ros::NodeHandle *n, tf::TransformListener *tflistener,
    bool use_dynamic_reconfig, string sub_odom_topic,
    string pub_current_pose_topic, string robot_name, string path_frame_id,
    string path_pose_frame_id, double traffic_direction)
    : n(n), tflistener(tflistener), use_dynamic_reconfig(use_dynamic_reconfig),
      sub_odom_topic(sub_odom_topic), traffic_direction(traffic_direction),
      pub_current_pose_topic(pub_current_pose_topic), robot_name(robot_name),
      path_frame_id(path_frame_id), path_pose_frame_id(path_pose_frame_id) {

  input_vel = 15;
  input_width = 3.00;
  input_max_acc = 3.00; // = normal car in the US

  step_size = 0.5;

  pub_current_pose =
      n->advertise<geometry_msgs::PoseStamped>(pub_current_pose_topic, 1000);
  sub_odom = n->subscribe<nav_msgs::Odometry>(
      sub_odom_topic, 1000, &OvertakingManeuver::odom_callback, this);
}

bool OvertakingManeuver::get_use_dynamic_reconfig() {
  return use_dynamic_reconfig;
}

double OvertakingManeuver::calculate_total_dis(double input_vel,
                                               double input_width,
                                               double input_max_acc) {
  return 2.4 * input_vel * sqrt(input_width / input_max_acc);
}

double OvertakingManeuver::calculate_total_time(double input_vel,
                                                double input_width,
                                                double input_max_acc) {
  return sqrt(3) * pow(input_width, 3 / 2) * sqrt(input_max_acc) /
             pow(input_vel, 2) +
         2.4 * sqrt(input_width) / sqrt(input_max_acc);
}

double OvertakingManeuver::calculate_x_at_t(double input_vel, double dis,
                                            double total_time, double time_t) {
  return input_vel * time_t +
         (input_vel * total_time - dis) * (-10 * pow((time_t / total_time), 3) +
                                           15 * pow((time_t / total_time), 4) -
                                           6 * pow((time_t / total_time), 5));
}

double OvertakingManeuver::calculate_y_at_t(double input_width,
                                            double total_time, double time_t) {
  return input_width +
         input_width * (-10 * pow((time_t / total_time), 3) +
                        15 * pow((time_t / total_time), 4) -
                        6 * pow((time_t / total_time), 5));
}

void OvertakingManeuver::odom_callback(
    const nav_msgs::Odometry::ConstPtr &odom) {
  current_pose.header.frame_id = odom->header.frame_id;
  current_pose.header.stamp = odom->header.stamp;

  current_pose.pose.position.x = odom->pose.pose.position.x;
  current_pose.pose.position.y = odom->pose.pose.position.y;
  current_pose.pose.position.z = odom->pose.pose.position.z;

  current_pose.pose.orientation.x = odom->pose.pose.orientation.x;
  current_pose.pose.orientation.y = odom->pose.pose.orientation.y;
  current_pose.pose.orientation.z = odom->pose.pose.orientation.z;
  current_pose.pose.orientation.w = odom->pose.pose.orientation.w;
}

geometry_msgs::PoseStamped
OvertakingManeuver::transform(geometry_msgs::PoseStamped pose_tmp) {
  geometry_msgs::PoseStamped pose_tmp_map;
  try {
    ros::Time now = ros::Time::now();
    tflistener->waitForTransform(robot_name + "/map", robot_name + "/odom", now,
                                 ros::Duration(3.0));

    // transform
    tflistener->transformPose(robot_name + "/map", pose_tmp, pose_tmp_map);

  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  return pose_tmp_map;
}

void OvertakingManeuver::rotate_path(nav_msgs::Path *path) {
  geometry_msgs::PoseStamped pose_tmp_map;
  geometry_msgs::PoseStamped pose_tmp_odom;
  pose_tmp_odom.header.frame_id = robot_name + "/odom";
  pose_tmp_odom.header.stamp = ros::Time::now();
  pose_tmp_odom.pose = current_pose.pose;
  pose_tmp_odom.pose.orientation.x = 0;
  pose_tmp_odom.pose.orientation.y = 0;
  pose_tmp_odom.pose.orientation.z = 0;
  pose_tmp_odom.pose.orientation.w = 1;
  pose_tmp_map = transform(pose_tmp_odom);

  tf::Pose pose;
  tf::poseMsgToTF(current_pose.pose, pose);
  double yaw_angle = tf::getYaw(pose.getRotation());
  double cos_yaw = cos(yaw_angle);
  double sin_yaw = sin(yaw_angle);

  // Rotation of each point
  for (int i = 0; i < path->poses.size(); i++) {
    double x = path->poses[i].pose.position.x;
    double y = path->poses[i].pose.position.y;
    double x_origin = x - pose_tmp_map.pose.position.x;
    double y_origin = y - pose_tmp_map.pose.position.y;
    double x_new = x_origin * cos_yaw - y_origin * sin_yaw;
    double y_new = x_origin * sin_yaw + y_origin * cos_yaw;
    x_new += pose_tmp_map.pose.position.x;
    y_new += pose_tmp_map.pose.position.y;
    path->poses[i].pose.position.x = x_new;
    path->poses[i].pose.position.y = y_new;
  }
}

// OUTPUTS
// Trajectory path: x(t) and y(t)
// D: Total x-direction distance
// T: Total time for maneuver
bool OvertakingManeuver::publish_trajectory(
    overtaking_maneuver::PublishOvertakingTrajectory::Request &req,
    overtaking_maneuver::PublishOvertakingTrajectory::Response &res) {

  if (!use_dynamic_reconfig) {
    this->input_vel = req.input_vel;
    this->input_width = req.input_width;
    this->input_max_acc = req.input_max_acc;
  }

  double total_dis = calculate_total_dis(input_vel, input_width, input_max_acc);
  double total_time =
      calculate_total_time(input_vel, input_width, input_max_acc);

  ROS_INFO("use_dynamic_reconfig %d vel %f, width %f, max_acc %f",
           use_dynamic_reconfig, input_vel, input_width, input_max_acc);
  ROS_INFO("d %f", total_dis);
  ROS_INFO("t %f", total_time);

  // init temp path in map frame
  nav_msgs::Path path_tmp;
  path_tmp.header.frame_id = path_frame_id;
  path_tmp.header.stamp = ros::Time::now();

  // init temp pose in odom frame
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header.frame_id = robot_name + "/odom";
  pose_tmp.header.stamp = ros::Time::now();
  pose_tmp.pose.orientation.x = 0;
  pose_tmp.pose.orientation.y = 0;
  pose_tmp.pose.orientation.z = 0;
  pose_tmp.pose.orientation.w = 1;

  // init temp pose in map frame
  geometry_msgs::PoseStamped pose_tmp_map;

  double y_t_0 = calculate_y_at_t(input_width, total_time, 0);

  double time = 0;
  while (time <= total_time) {

    double x = calculate_x_at_t(input_vel, total_dis, total_time, time);
    double y = calculate_y_at_t(input_width, total_time, time);

    // odom frame
    pose_tmp.pose.position.x = (x) + current_pose.pose.position.x;
    pose_tmp.pose.position.y =
        (((-1) * y + y_t_0) * traffic_direction) + current_pose.pose.position.y;
    pose_tmp.pose.position.z = (0) + current_pose.pose.position.z;

    pose_tmp_map = transform(pose_tmp);

    // push_back
    pose_tmp.header.frame_id = path_pose_frame_id;
    path_tmp.poses.push_back(pose_tmp_map);
    pose_tmp.header.frame_id = robot_name + "/odom";

    time = time + step_size;
  }

  time = 0;
  while (time <= total_time) {
    double x = calculate_x_at_t(input_vel, total_dis, total_time, time);
    double y = calculate_y_at_t(input_width, total_time, time);

    // odom frame
    pose_tmp.pose.position.x = (x) + current_pose.pose.position.x + total_dis;
    pose_tmp.pose.position.y =
        ((y)*traffic_direction) + current_pose.pose.position.y;
    pose_tmp.pose.position.z = (0) + current_pose.pose.position.z;

    pose_tmp_map = transform(pose_tmp);

    // push_back
    pose_tmp.header.frame_id = path_pose_frame_id;
    path_tmp.poses.push_back(pose_tmp_map);
    pose_tmp.header.frame_id = robot_name + "/odom";

    time = time + step_size;
  }

  double distance = step_size;
  while (distance <= total_dis / 5) {

    pose_tmp.pose.position.x += distance;

    pose_tmp_map = transform(pose_tmp);

    // push_back
    pose_tmp.header.frame_id = path_pose_frame_id;
    path_tmp.poses.push_back(pose_tmp_map);
    pose_tmp.header.frame_id = robot_name + "/odom";

    distance = distance + step_size;
  }

  // rotate
  rotate_path(&path_tmp);

  // finish
  path_tmp.header.frame_id = path_frame_id;
  res.path_custom_frame = path_tmp;
  path_tmp.header.frame_id = robot_name + "/map";
  res.path_map_frame = path_tmp;
  path_tmp.poses.clear();

  res.finished = true;
  return res.finished;
}

void OvertakingManeuver::dynamic_config_callback(
    overtaking_maneuver::OvertakingManeuverInputsConfig &config,
    uint32_t level) {
  input_vel = config.input_vel;
  input_width = config.input_width;
  input_max_acc = config.input_max_acc;
}
