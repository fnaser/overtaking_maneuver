#include "overtaking_maneuver/overtaking_maneuver.h"

OvertakingManeuver::~OvertakingManeuver() {}

OvertakingManeuver::OvertakingManeuver() {}

OvertakingManeuver::OvertakingManeuver(
    ros::NodeHandle *n, string sub_user_input_topic, string sub_odom_topic,
    string pub_path_topic, string pub_path_topic_test,
    string pub_current_pose_topic, string robot_name, string path_frame_id,
    string path_pose_frame_id)
    : n(n), sub_user_input_topic(sub_user_input_topic),
      sub_odom_topic(sub_odom_topic), pub_path_topic(pub_path_topic),
      pub_path_topic_test(pub_path_topic_test),
      pub_current_pose_topic(pub_current_pose_topic), robot_name(robot_name),
      path_frame_id(path_frame_id), path_pose_frame_id(path_pose_frame_id) {
  input_vel = 15;
  input_width = 3.00;
  input_max_acc = 3.00; // = normal car in the US

  update_odom = false;
  time_step_size = 0.5;

  pub_trajectory = n->advertise<nav_msgs::Path>(pub_path_topic, 1000);
  pub_current_pose =
      n->advertise<geometry_msgs::PoseStamped>(pub_current_pose_topic, 1000);
  pub_trajectory_test = n->advertise<nav_msgs::Path>(pub_path_topic_test, 1000);
  sub_odom =
      n->subscribe<nav_msgs::Odometry>(sub_odom_topic, 1000, &OvertakingManeuver::odom_callback, this);
  sub_user_input = n->subscribe<std_msgs::Bool>(sub_user_input_topic, 1000,
                                               &OvertakingManeuver::user_input_callback, this);

//  dynamic_reconfigure::Server<
//      overtaking_maneuver::OvertakingManeuverInputsConfig> server;
//  dynamic_reconfigure::Server<
//      overtaking_maneuver::OvertakingManeuverInputsConfig>::CallbackType f;
//  f = boost::bind(&OvertakingManeuver::dynamic_config_callback, _1, _2);
//  server.setCallback(f);
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

void OvertakingManeuver::user_input_callback(
    const std_msgs::Bool::ConstPtr &msg) {
  update_odom = msg->data;
}

void OvertakingManeuver::odom_callback(
    const nav_msgs::Odometry::ConstPtr &odom) {
  if (update_odom) {
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
}

void OvertakingManeuver::rotate_path(nav_msgs::Path *path,
                                     tf::TransformListener *tflistener) {
  geometry_msgs::PoseStamped pose_tmp_map;
  geometry_msgs::PoseStamped pose_tmp_odom;
  pose_tmp_odom.header.frame_id = robot_name + "/odom";
  pose_tmp_odom.header.stamp = ros::Time::now();
  pose_tmp_odom.pose = current_pose.pose;
  pose_tmp_odom.pose.orientation.x = 0;
  pose_tmp_odom.pose.orientation.y = 0;
  pose_tmp_odom.pose.orientation.z = 0;
  pose_tmp_odom.pose.orientation.w = 1;
  tflistener->transformPose(robot_name + "/map", pose_tmp_odom, pose_tmp_map);

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

void OvertakingManeuver::publish_trajectory(tf::TransformListener *tflistener) {

  // OUTPUTS
  // Trajectory path: x(t) and y(t)
  // D: Total x-direction distance
  // T: Total time for maneuver

  double total_dis = calculate_total_dis(input_vel, input_width, input_max_acc);
  double total_time =
      calculate_total_time(input_vel, input_width, input_max_acc);
  ROS_INFO("vel %f, width %f, max_acc %f", input_vel, input_width,
           input_max_acc);
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

  double y_t_0 = calculate_y_at_t(input_width, total_time, 0);
  double offset_x = total_dis;

  double time = 0;
  while (time <= total_time) {

    double x = calculate_x_at_t(input_vel, total_dis, total_time, time);
    double y = calculate_y_at_t(input_width, total_time, time);

    // Odom frame
    pose_tmp.pose.position.x =
        (x - total_dis) + current_pose.pose.position.x + offset_x;
    pose_tmp.pose.position.y =
        ((-1) * y + y_t_0) + current_pose.pose.position.y;
    pose_tmp.pose.position.z = (0) + current_pose.pose.position.z;

    // Transform
    geometry_msgs::PoseStamped pose_tmp_map;
    tflistener->transformPose(robot_name + "/map", pose_tmp, pose_tmp_map);

    // Map frame
    pose_tmp = pose_tmp_map;

    pose_tmp.header.frame_id = path_pose_frame_id;
    // path_tmp.poses.push_back(pose_tmp);
    path_tmp.poses.push_back(pose_tmp_map);
    pose_tmp.header.frame_id = robot_name + "/odom";

    time = time + time_step_size;
  }

  time = 0;
  while (time <= total_time) {
    double x = calculate_x_at_t(input_vel, total_dis, total_time, time);
    double y = calculate_y_at_t(input_width, total_time, time);

    pose_tmp.pose.position.x = (x) + current_pose.pose.position.x + offset_x;
    pose_tmp.pose.position.y = (y) + current_pose.pose.position.y;
    pose_tmp.pose.position.z = (0) + current_pose.pose.position.z;

    // Transform
    geometry_msgs::PoseStamped pose_tmp_map;
    tflistener->transformPose(robot_name + "/map", pose_tmp, pose_tmp_map);

    // Map frame
    pose_tmp = pose_tmp_map;

    pose_tmp.header.frame_id = path_pose_frame_id;
    // path_tmp.poses.push_back(pose_tmp);
    path_tmp.poses.push_back(pose_tmp_map);
    pose_tmp.header.frame_id = robot_name + "/odom";

    time = time + time_step_size;
  }

  rotate_path(&path_tmp, tflistener);

  pub_trajectory.publish(path_tmp);
  path_tmp.header.frame_id = robot_name + "/map";
  pub_trajectory_test.publish(path_tmp);
  // ROS_INFO("path.poses.size() %d", path_tmp.poses.size());
  path_tmp.poses.clear();
}

void OvertakingManeuver::dynamic_config_callback(
    overtaking_maneuver::OvertakingManeuverInputsConfig &config,
    uint32_t level) {
  input_vel = config.input_vel;
  input_width = config.input_width;
  input_max_acc = config.input_max_acc;
}
