#include "overtaking_maneuver/overtaking_maneuver.h"

std::string node_name = "overtaking_maneuver_node";
std::string sub_user_input_topic = "update_odom_user_input";
std::string sub_odom_topic = "odom";
std::string pub_path_topic = "overtaking_path";
std::string pub_path_topic_test = "overtaking_path_test";
std::string pub_current_pose_topic = "current_pose";
std::string robot_name = "catvehicle";
std::string path_frame_id = "map";
std::string path_pose_frame_id = "";

int main(int argc, char **argv) {
  ros::init(argc, argv, node_name);

  ros::NodeHandle n;

  OvertakingManeuver *om = new OvertakingManeuver(
      &n, sub_user_input_topic, sub_odom_topic, pub_path_topic,
      pub_path_topic_test, pub_current_pose_topic, robot_name, path_frame_id,
      path_pose_frame_id);

  tf::TransformListener tflistener;

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    tf::StampedTransform transform;
    try {
      tflistener.lookupTransform(robot_name + "/map", robot_name + "/odom",
                                 ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    if (om->update_odom) {
      om->publish_trajectory(&tflistener);
      om->update_odom = false; // pub only once
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
