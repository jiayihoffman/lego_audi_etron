# Audi RS Q e-tron

LEGO Audi RS Q e-tron (Technic #42160) is a mobile robot with two front wheels for steering and two rear wheels for forward and backward motion. 

Because each pair of wheels (steering and traction) is controlled by a single interface, a bicycle steering controller is used, in which two joints, virtual_rear_wheel_joint and virtual_front_wheel_joint, are controlled to drive the carlike robot. 

For more information about the ROS2 Bicycle Steering Controller, please see the documentation on [control.ros.org](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model) and [steering_controllers](https://control.ros.org/humble/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html#steering-controllers-library-userdoc). The second article provides information on the controller's parameters and subscribed/published topics. 

## Build

### Docker
The Docker image provides a ROS2 environment with all dependencies. To build the Docker image, use this command. 

```bash
docker build -t audi_etron .
```

Note: I could not build the image on the Raspberry Pi due to limited compute resources. I had to build the image on my Linux dev machine, push it to the Docker Hub, and pull it onto the Pi. Therefore, I created a script for building and pushing: 
`./scripts/publish-docker.sh`. 

### Local
if not using Docker, follow these instructions to build the package:

1. Build and install `SimpleBLE`, which provides Bluetooth support for the LEGO hub and motors.
* The source code for SimpleBLE is at: https://simpleble.readthedocs.io/en/latest/overview.html
* For the build and installation instructions, please check [Build_SimpleBLE.md](./Build_SimpleBLE.md)
2. build the audi_etron ROS2 package in the workspace directory. 
   ```bash
   colcon build --packages-select=audi_etron
   source install/setup.bash 
   ```

## Quick Play:
Run the robot and view it in **RViz** and in the real world. 

### Docker

#### Start the robot
Turn on the LEGO Hub, and immediately start the "audi_etron" docker container. The container's launch file loads and starts the robot hardware and controllers.

```bash
./scripts/start-robot.sh
```

Use the --pull flag to force a pull of the image from the Docker Hub repository. Use the "pull" option when the image has been updated on the hub. If the image is private, log in to Docker Hub first.

```bash
docker login
./scripts/start-robot.sh --pull
```

About Docker container's D-Bus and Bluetooth access:
- `--privileged`: Required to bypass AppArmor restrictions and access D-Bus/Bluetooth hardware
- `-v /var/run/dbus:/var/run/dbus`: Mounts the host's D-Bus socket so SimpleBLE can communicate with BlueZ
- SimpleBLE uses D-Bus to communicate with the BlueZ daemon for Bluetooth Low Energy operations

#### Visualization

To view the robot in RViz, I use my Linux dev machine. The `ROS_DOMAIN_ID` environment variable must be the same between the dev machine and the docker. It controls who can access the data published by the robot running in the docker.

```bash
ros2 launch audi_etron view_robot_rviz2.launch.py
```

### Local
Turn on the LEGO Hub, and start the robot with RViz. The launch file loads and starts the robot hardware, controllers and opens RViz.
```bash
ros2 launch audi_etron lego_audi_etron.launch.py remap_odometry_tf:=true
```

Note, to see only the robot description, use the following command. The "-d" option launches the robot in the debug log level.
```bash
ros2 launch audi_etron view_robot.launch.py -d
```

### Hardware Interfaces and Controllers
Let's introspect the control system before driving the robot:

1. Check if the hardware interfaces are loaded properly:
   ```bash
   ros2 control list_hardware_interfaces
   ```

   We should see this: 
   ```
   command interfaces
      bicycle_steering_controller/angular/velocity [available] [unclaimed]
      bicycle_steering_controller/linear/velocity [available] [unclaimed]
      virtual_front_wheel_joint/position [available] [claimed]
      virtual_rear_wheel_joint/velocity [available] [claimed]
   state interfaces
      virtual_front_wheel_joint/position
      virtual_rear_wheel_joint/position
      virtual_rear_wheel_joint/velocity
   ```
   The [claimed] marker on command interfaces means that a controller has access to those interfaces.

2. Check if controllers are running:
   ```bash
   ros2 control list_controllers
   ```

   We should see:
   ```
   joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
   bicycle_steering_controller bicycle_steering_controller/BicycleSteeringController  active
   ```

### Send commands for the car to circling

If everything is fine, now we can send commands to drive the robot. 

Here is a quick command that publishes the stamped twist messages to the robot's /cmd_vel topic.  

```bash
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

To use a joystick (gamepad) controlling the robot, launch the `teleop_twist_joy` node.

Each gamepad has an enable button. Press it while twisting the joystick to control the robot. On the PS5, it is the "PS" button between the two joysticks.  

```bash
# Install the ROS2 teleop-twist-joy package
sudo apt install ros-${ROS_DISTRO}-teleop-twist-joy

# launch the teleop-twist-joy 
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' publish_stamped_twist:=true
```
