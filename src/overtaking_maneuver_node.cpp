#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

std::string node_name = "overtaking_maneuver_node";
std::string pub_vel_cmd = "overtaking_vel_cmd";

// INPUTS
// V (>= 5 m/s): Initial and final velocity [m/s]
// W: Width of the lane or of the diversion / total y-direction distance [m]
// A: Magnitude of the maximal resultant acceleration of ego vehicle [m/s^2]
double input_vel = 2.77; // = 10 km/s
double input_width = 5; //
double input_max_acc = 3; // normal car in the US

double input_vel_obstacle = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, node_name);

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>(pub_vel_cmd, 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "Hello World";

    // OUTPUTS
    // x(t) and y(t)
    // D: Total x-direction distance
    // T: Total time for maneuver
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
