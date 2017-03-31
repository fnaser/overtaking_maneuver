# overtaking_maneuver

ROS Node to calculate the optimal trajectory for an autonomous vehicle to overtake a slower moving vehicle.

Based on:
- T. Shamir, "How should an autonomous vehicle overtake a slower moving vehicle: design and analysis of an optimal trajectory," in IEEE Transactions on Automatic Control, vol. 49, no. 4, pp. 607-610, April 2004. doi: 10.1109/TAC.2004.825632
- http://cps-vo.org/group/CATVehicleTestbed

# Installation Indigo

 - http://releases.ubuntu.com/14.04/
 - http://wiki.ros.org/indigo/Installation/Ubuntu
```
sudo apt-get install git
sudo apt-get install chromium-browser
```
 - http://cps-vo.org/node/26602 (https://github.com/fnaser/overtaking_maneuver/blob/master/pdfs/Installation%20_%20CPS-VO.pdf)
 - http://cps-vo.org/node/26591 (https://github.com/fnaser/overtaking_maneuver/blob/master/pdfs/Compile%20the%20release%20_%20CPS-VO.pdf)
```
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/catvehicle.git
~/catvehicle_ws/src/$ git clone https://github.com/fnaser/overtaking_maneuver.git
```
 - ~/.bashrc
```
source /opt/ros/indigo/setup.bash
source ~/catvehicle_ws/devel/setup.bash
```
 - download Gazebo files https://bitbucket.org/osrf/gazebo_models/downloads/
 - mv to ~/.gazebo/models

# Installation Kinetic

 - http://releases.ubuntu.com/16.04/
 - http://wiki.ros.org/kinetic/Installation/Ubuntu

# How to run the simulaton

```
roscore
rqt
roslaunch catvehicle 2_lidars_2_cars.launch
roslaunch overtaking_maneuver overtaking_maneuver.launch
```
