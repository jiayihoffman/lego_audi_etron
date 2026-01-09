# Audi RS Q e-tron

LEGO Audi RS Q e-tron (Technic #42160) is a mobile robot with two front wheels for steering and two rear wheels for forward and backward motion. 

Because each pair of wheels (steering and traction) is controlled by a single interface, a bicycle steering controller is used, in which two joints, virtual_rear_wheel_joint and virtual_front_wheel_joint, are controlled to drive the carlike robot. 

For more information about the ROS2 Bicycle Steering Controller, please see the documentation on [control.ros.org](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model).


## Build

### Docker
The Docker image provides a ROS2 environment with all dependencies. To build the Docker image, use the command:

```
docker build -t audi_etron_image .
```

### Standard
if not using Docker, follow these instructions to build the package:

1. Build and install `SimpleBLE`, which provides Bluetooth support for the LEGO hub and motors.
* The source code for SimpleBLE is at: https://simpleble.readthedocs.io/en/latest/overview.html
* I prepared the build instructions for SimpleBLE in the file "Build_SimpleBLE.md"
2. build the audi_etron ROS2 package in the workspace directory. 
```
colcon build --packages-select=audi_etron
source install/setup.bash 
```

## Quick Play:
Run the robot and view it in **RViz** and in the real world.

### Docker
Start the audi_etron docker container:
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

### Send a command for the car to circling

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
