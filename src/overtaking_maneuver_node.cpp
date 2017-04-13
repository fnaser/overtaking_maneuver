#include "overtaking_maneuver/overtaking_maneuver.h"

std::string node_name = "overtaking_maneuver_node";
std::string sub_odom_topic = "odom";
std::string pub_current_pose_topic = "current_pose";
std::string robot_name = "catvehicle";
std::string path_frame_id = "map";
std::string path_pose_frame_id = "";

void test_tf(tf::TransformListener *tflistener) {
  tf::StampedTransform transform;
  try {
    tflistener->lookupTransform(robot_name + "/map", robot_name + "/odom",
                               ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, node_name);

  ros::NodeHandle n;

  bool use_dynamic_reconfig;
  double traffic_direction;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("use_dynamic_reconfig", use_dynamic_reconfig,
                             bool(true));
  private_node_handle_.param("robot_name", robot_name,
                             std::string("catvehicle"));
  private_node_handle_.param("traffic_direction", traffic_direction, double(1));

  tf::TransformListener tflistener;
  test_tf(&tflistener);

  OvertakingManeuver *om = new OvertakingManeuver(
      &n, &tflistener, use_dynamic_reconfig, sub_odom_topic,
      pub_current_pose_topic, robot_name, path_frame_id, path_pose_frame_id,
      traffic_direction);

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
    test_tf(&tflistener);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
