# Audi RS Q e-tron

LEGO Audi RS Q e-tron (Technic #42160) is a mobile robot with two front wheels for steering and two rear wheels for forward and backward motion. 

Because each pair of wheels (steering and traction) is controlled by a single interface, a bicycle steering controller is used, in which two joints, virtual_rear_wheel_joint and virtual_front_wheel_joint, are controlled to drive the carlike robot. 

For more information about the ROS2 Bicycle Steering Controller, please see the documentation on [control.ros.org](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model) and [steering_controllers](https://control.ros.org/humble/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html#steering-controllers-library-userdoc). The second article provides information on the controller's parameters and subscribed/published topics. 

## Build

### Docker
The Docker image provides a ROS2 environment with all dependencies. To build the Docker image, use this command. 

```
docker build -t audi_etron .
```

Note: I could not build the image on the Raspberry Pi due to limited compute resource. I had to build the image on my Linux dev machine, push the image to the Docker Hub, and pull the image onto the Pi. Therefore, I created the script for build and push. 
`./scripts/publish-docker.sh`. 

### Local
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
Turn on the Lego Hub, and immediately start the audi_etron docker container:

```
# start the robot docker container. Checks the local repository first. If not exist, pull the image from the Docker Hub. 
./scripts/start-robot.sh

# use `--pull` flag to force the pull. Use it when the image has been updated in the Docker hub.
./scripts/start-robot.sh --pull
```

View the robot in RViz on the Linux dev machine. Make sure the `ROS_DOMAIN_ID` environment variable on the dev machine matches the one set on the robot. This parameter controls who can access the data published by the robot running on the Raspberry Pi.

```
ros2 launch audi_etron view_robot_rviz2.launch.py
```

About D-Bus and Bluetooth access:
- `--privileged`: Required to bypass AppArmor restrictions and access D-Bus/Bluetooth hardware
- `-v /var/run/dbus:/var/run/dbus`: Mounts the host's D-Bus socket so SimpleBLE can communicate with BlueZ
- SimpleBLE uses D-Bus to communicate with the BlueZ daemon for Bluetooth Low Energy operations

### Local

```
# view the robot in RViz
ros2 launch audi_etron view_robot.launch.py

# "-d" for ros2 launch in the debug log level
ros2 launch audi_etron view_robot.launch.py -d

# Turn on the Lego Hub, and start the robot with RViz (default: gui:=true)
ros2 launch audi_etron carlikebot.launch.py remap_odometry_tf:=true

ros2 control list_hardware_interfaces
ros2 control list_controllers
```

### Send commands for the car to circling

Here is a quick command that publishes the stamped twist messages to the robot's /cmd_vel topic.  

```
ros2 topic pub --rate 30 /cmd_vel geometry_msgs/msg/TwistStamped "
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

To use a joystick (gamepad) to control the robot, launch the teleop_twist_joy node.

Each gamepad has an enable button. Press it while twisting the joystick to control the robot. On the PS5, it is the "PS" button between the two joysticks.  

```
# Install the ROS2 teleop-twist-joy package
sudo apt install ros-${ROS_DISTRO}-teleop-twist-joy

# launch the teleop-twist-joy 
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' publish_stamped_twist:=true
```
