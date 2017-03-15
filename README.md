# overtaking_maneuver
ROS Node to calculate the optimal trajectory for an autonomous vehicle to overtake a slower moving vehicle.

Based on:
- T. Shamir, "How should an autonomous vehicle overtake a slower moving vehicle: design and analysis of an optimal trajectory," in IEEE Transactions on Automatic Control, vol. 49, no. 4, pp. 607-610, April 2004. doi: 10.1109/TAC.2004.825632
- http://cps-vo.org/group/CATVehicleTestbed

# How to run the simulaton

```
roscore
rqt
rosrun rviz rviz
roslaunch catvehicle catvehicle_skidpan.launch
gzclient
```
