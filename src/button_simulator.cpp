#include "ros/ros.h"
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "button_simulator");
  ros::NodeHandle n;

  int rate;

  std_msgs::Bool button_state_automode, button_state_emergency,
      button_state_reverse;
  std_msgs::String button_status;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(100));

  ros::Publisher pub_automode =
      n.advertise<std_msgs::Bool>("button_state_automode", 1);
  ros::Publisher pub_emergency =
      n.advertise<std_msgs::Bool>("button_state_emergency", 1);
  ros::Publisher pub_reverse =
      n.advertise<std_msgs::Bool>("button_state_reverse", 1);
  ros::Publisher pub_button_status =
      n.advertise<std_msgs::String>("button_status", 1);
  ros::Rate r(rate);

  while (n.ok()) {
    // autonomous driving mode: "on"
    button_state_automode.data = true;
    button_state_emergency.data = false;
    button_state_reverse.data = false;
    button_status.data = "Auto";

    pub_automode.publish(button_state_automode);
    pub_emergency.publish(button_state_emergency);
    pub_reverse.publish(button_state_reverse);
    pub_button_status.publish(button_status);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
