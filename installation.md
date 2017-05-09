# Installation
```
sudo apt-get install git
sudo apt-get install chromium-browser
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/catvehicle.git
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/overtaking_maneuver.git
~/.bashrc:
source /opt/ros/<ros>/setup.bash
source ~/catvehicle_ws/devel/setup.bash
```

# Installation Ubuntu 14 and ROS Indigo

 - http://releases.ubuntu.com/14.04/
 - http://wiki.ros.org/indigo/Installation/Ubuntu
 - `ros-indigo-fake-localization`
 - http://cps-vo.org/node/26602
 - http://cps-vo.org/node/26591
 - http://gazebosim.org/tutorials?tut=install_ubuntu
 - download Gazebo files https://bitbucket.org/osrf/gazebo_models/downloads/
 - mv files to ~/.gazebo/models

# How to run the simulaton

```
roscore
rqt
roslaunch catvehicle 2_lidars_2_cars.launch
roslaunch catvehicle hector_slam.launch
roslaunch overtaking_maneuver overtaking_maneuver.launch
only run during simulation (otherwise potential safety hazard on hardware platform) roslaunch overtaking_maneuver button_simulator.launch
```
