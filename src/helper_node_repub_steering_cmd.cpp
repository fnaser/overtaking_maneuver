#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <string>

std::string node_name = "helper_node_repub_steering_cmd";
std::string sub_steer_topic = "cmd_steer";
std::string pub_vel_topic = "cmd_vel";

// Current steering angle [rad]
double current_steer_angle_rad;
// Const vel cmd
double const_vel = 0.5;

void steer_callback(const geometry_msgs::Twist::ConstPtr &steer) {
  current_steer_angle_rad = steer->angular.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, node_name);

  ros::NodeHandle n;
  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>(pub_vel_topic, 1000);
  ros::Subscriber sub_steer =
      n.subscribe(sub_steer_topic, 1000, steer_callback);
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    geometry_msgs::Twist vel_cmd_tmp;
    vel_cmd_tmp.linear.x = const_vel;
    vel_cmd_tmp.angular.z = current_steer_angle_rad;
    pub_vel.publish(vel_cmd_tmp);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
