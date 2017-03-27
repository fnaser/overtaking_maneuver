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

  bool update_odom, use_dynamic_reconfig;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("update_odom", update_odom, bool(true));
  private_node_handle_.param("use_dynamic_reconfig", use_dynamic_reconfig,
                             bool(true));

  tf::TransformListener tflistener;

  OvertakingManeuver *om = new OvertakingManeuver(
      &n, &tflistener, update_odom, use_dynamic_reconfig, sub_user_input_topic,
      sub_odom_topic, pub_path_topic, pub_path_topic_test,
      pub_current_pose_topic, robot_name, path_frame_id, path_pose_frame_id);

  // http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
  ros::ServiceServer ss =
      n.advertiseService("publish_overtaking_trajectory",
                         &OvertakingManeuver::publish_trajectory, om);

  ros::Rate loop_rate(10);

  // http://wiki.ros.org/ROSNodeTutorialC%2B%2B
  dynamic_reconfigure::Server<
      overtaking_maneuver::OvertakingManeuverInputsConfig> server;
  dynamic_reconfigure::Server<
      overtaking_maneuver::OvertakingManeuverInputsConfig>::CallbackType f;
  f = boost::bind(&OvertakingManeuver::dynamic_config_callback, om, _1, _2);
  server.setCallback(f);

  while (ros::ok()) {

    tf::StampedTransform transform;
    try {
      tflistener.lookupTransform(robot_name + "/map", robot_name + "/odom",
                                 ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
