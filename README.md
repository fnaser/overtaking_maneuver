# overtaking_maneuver

ROS Node to calculate the optimal trajectory for an autonomous vehicle to overtake a slower moving vehicle.

Based on:
- T. Shamir, "How should an autonomous vehicle overtake a slower moving vehicle: design and analysis of an optimal trajectory," in IEEE Transactions on Automatic Control, vol. 49, no. 4, pp. 607-610, April 2004. doi: 10.1109/TAC.2004.825632
- http://cps-vo.org/group/CATVehicleTestbed

# Installation
```
sudo apt-get install git
sudo apt-get install chromium-browser
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/catvehicle.git
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/overtaking_maneuver.git
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/obstaclestopper.git
~/.bashrc:
source /opt/ros/<ros>/setup.bash
source ~/catvehicle_ws/devel/setup.bash
```

# Installation Indigo (recommended)

 - http://releases.ubuntu.com/14.04/
 - http://wiki.ros.org/indigo/Installation/Ubuntu
 - http://cps-vo.org/node/26602
 - http://cps-vo.org/node/26591
 - http://gazebosim.org/tutorials?tut=install_ubuntu
 - download Gazebo files https://bitbucket.org/osrf/gazebo_models/downloads/
 - mv to ~/.gazebo/models

# Installation Kinetic

 - http://releases.ubuntu.com/16.04/
 - http://wiki.ros.org/kinetic/Installation/Ubuntu
```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control libpcap0.8-dev
```
 - https://github.com/ros-simulation/gazebo_ros_pkgs
 - http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros

# How to run the simulaton

```
roscore
rqt
roslaunch catvehicle 2_lidars_2_cars.launch
roslaunch catvehicle hector_slam.launch
roslaunch overtaking_maneuver overtaking_maneuver.launch
```
