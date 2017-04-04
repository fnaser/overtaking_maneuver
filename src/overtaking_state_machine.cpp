#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <overtaking_maneuver/PublishOvertakingTrajectory.h>

std::string node_name = "overtaking_state_machine";
std::string sub_obstacle_topic = "store_path_before_overtaking";
std::string sub_way_point = "way_point_percentage";
std::string service_name = "publish_overtaking_trajectory";

std::string pub_original_path_topic = "route_plan";
std::string pub_path_topic_custom = "overtaking_path";
std::string pub_path_topic_overtaking = "overtaking_path_test";

nav_msgs::Path latest_path;
ros::ServiceClient client;
ros::Publisher pub_original_trajectory, pub_custom_trajectory,
    pub_overtaking_trajectory;
bool published_new_path = false;

double input_vel, input_width, input_max_acc;

void latest_way_point_callback(const std_msgs::Float64::ConstPtr &percentage) {
  if (percentage->data > 0.9) {
    pub_original_trajectory.publish(latest_path);
    published_new_path = false;
    ROS_INFO("overtaking maneuver over");
  }
}

void obstacle_callback(const nav_msgs::Path::ConstPtr &path) {
  if (!published_new_path) {
    latest_path = *path;
    ROS_INFO("obstacle in the way detected");
    overtaking_maneuver::PublishOvertakingTrajectory srv;
    srv.request.input_vel = input_vel;
    srv.request.input_width = input_width;
    srv.request.input_max_acc = input_max_acc;

    ROS_INFO("call service");
    if (client.call(srv)) {

      pub_custom_trajectory.publish(srv.response.path_custom_frame);
      pub_overtaking_trajectory.publish(srv.response.path_map_frame);

      published_new_path = true;
    } else {
      ROS_ERROR("failed to call service");
      // return 1;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("input_vel", input_vel, double(7));
  private_node_handle_.param("input_width", input_width, double(5));
  private_node_handle_.param("input_max_acc", input_max_acc, double(3));

  ros::Subscriber sub_percentage = n.subscribe<std_msgs::Float64>(
      sub_way_point, 1000, &latest_way_point_callback);
  ros::Subscriber sub_obstacle =
      n.subscribe<nav_msgs::Path>(sub_obstacle_topic, 1000, &obstacle_callback);
  pub_original_trajectory =
      n.advertise<nav_msgs::Path>(pub_original_path_topic, 1000);
  pub_custom_trajectory =
      n.advertise<nav_msgs::Path>(pub_path_topic_custom, 1000);
  pub_overtaking_trajectory =
      n.advertise<nav_msgs::Path>(pub_path_topic_overtaking, 1000);

  client = n.serviceClient<overtaking_maneuver::PublishOvertakingTrajectory>(
      service_name);

  ros::spin();

  return 0;
}
