# overtaking_maneuver
ROS Node to calculate the optimal trajectory for an autonomous vehicle to overtake a slower moving vehicle.

Based on:
- 10.1109/TAC.2004.825632
- http://cps-vo.org/group/CATVehicleTestbed

# How to run the simulaton

```
roscore
rqt
rosrun rviz rviz
roslaunch catvehicle catvehicle_skidpan.launch
gzclient
```
