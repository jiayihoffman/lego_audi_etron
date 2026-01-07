# audi_etron

   *CarlikeBot*, or ''Carlike Mobile Robot'', is a simple mobile base with bicycle drive.
   The robot has two wheels in the front that steer the robot and two wheels in the back that power the robot forward and backwards. However, since each pair of wheels (steering and traction) are controlled by one interface, a bicycle steering model is used.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_11/doc/userdoc.html).


## Build

### Docker

```
# build the docker image "audi_etron_image"
docker build -t audi_etron_image .
```

### Standard
if not using Docker, follow these instruction:

1. Build and Install `SimpleBLE`, which provides Bluetooth support for the LEGO hub. 
* https://simpleble.readthedocs.io/en/latest/overview.html
* build instruction is at "Build_SimpleBLE.md"
2. build the ROS2 packages in the `lego_ws` directory. 
* The test requires "ros2_control_demo_testing" package to be copied from "ros2_control_demos" to the workspace directory. 

```
colcon build
source install/setup.bash 

colcon test
```

## Quick Play:
Run the robot and view it in **RViz**

### Docker

```
docker run -it --rm --network=host \
    -v /var/run/dbus:/var/run/dbus \
    audi_etron_image
```

About D-Bus socket mount: 
SimpleBLE uses D-Bus to communicate with BlueZ. `-v /var/run/dbus:/var/run/dbus` allows SimpleBLE to communicate with the BlueZ daemon via D-Bus. Without this mount, the container can't access the host's D-Bus socket.

### Standard

```
# view the robot
ros2 launch audi_etron view_robot.launch.py

# "-d" for ros2 launch in the debug log level
ros2 launch audi_etron view_robot.launch.py -d

# Start the robot
ros2 launch audi_etron carlikebot.launch.py remap_odometry_tf:=true

ros2 control list_hardware_interfaces
ros2 control list_controllers
```

Send a command for the car to circling in RViz
```
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
