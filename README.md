# audi_etron

   *CarlikeBot*, or ''Carlike Mobile Robot'', is a simple mobile base with bicycle drive.
   The robot has two wheels in the front that steer the robot and two wheels in the back that power the robot forward and backwards. However, since each pair of wheels (steering and traction) are controlled by one interface, a bicycle steering model is used.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_11/doc/userdoc.html).

## Build and Test
run this in the `lego_ws` directory. The test requires "ros2_control_demo_testing" package to be copied to the workspace. 
```
colcon build
source install/setup.bash 

colcon test
```

## Quick Play:

```
# view the robot
ros2 launch audi_etron view_robot.launch.py

# Start the robot
ros2 launch audi_etron carlikebot.launch.py remap_odometry_tf:=true

ros2 control list_hardware_interfaces
ros2 control list_controllers

# send a command for the car to circling in *RViz*
ros2 topic pub --rate 30 /bicycle_steering_controller/reference geometry_msgs/msg/TwistStamped "
twist:
   linear:
      x: 1.0
      y: 0.0
      z: 0.0
   angular:
      x: 0.0
      y: 0.0
      z: 0.8"

```
