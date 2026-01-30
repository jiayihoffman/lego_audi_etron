# LEGO Technic Audi RS Q e-tron

LEGO Technic Audi RS Q e-tron 42160 is a model of the 2022 Audi RS Q e-tron Dakar rally car. It features many realistic details, including individual suspension on each of the car’s 4 wheels and new Technic wheel elements created specifically for this model to reflect the full-sized Audi’s wheel design.

Like a real-world car, the LEGO Audi e-tron has front-wheel steering and all-wheel drive. It is powered by three Control+ motors, connected to the ports of the Technic Hub.

## ROS2 Control of LEGO Technic Audi RS Q e-tron
This project creates a ros2_control hardware component for the LEGO Technic Audi RS Q e-tron. 

### What is ROS2 Control

ros2_control is a robot control framework in ROS 2 that provides a hardware abstraction layer for controlling robot actuators, sensors, and hardware interfaces in a modular and efficient way. 

ros2_control enhances performance and offers real-time capabilities rather than multiple processes collaborating through messages and topics. Besides, it promotes standardization and modular robot control. It supports various hardware interfaces through the hardware abstraction layer and enables a seamless transition between simulated and different hardware implementations with little code change. 

Developers can reuse existing controllers instead of writing their own from scratch. In fact, much of the robot control logic has already been developed by others, so the pre-built ROS2 controllers can be utilized as is in most use cases.

Controller Manager is the main component of the ros2_control framework. It manages the lifecycle of controllers, provides access to hardware interfaces, and offers services to the ROS-echo system.

### Controllers for Wheeled Mobile Robots

For wheeled mobile robots, ros2_control framework offers the following types of controllers. 
* Differential Drive Controller: 
   * Controller for mobile robots with differential drive, which has two wheels, each of which is driven independently.
* Steering Controllers:
   * Bicycle - with one steering and one drive joints;
   * Tricylce - with one steering and two drive joints;
   * Ackermann - with two steering and two drive joints.
* Mecanum Drive Controllers: 
   * Controller for mobile robot with four mecanum wheels allowing the robot to move sideways, spin, and drive in any direction by controlling each wheel independently. 

### Bicycle Steering Controllers for LEGO Audi RS Q e-tron
This LEGO Audi RS Q e-tron is a car-like robot. It has steerable front wheels and all-wheel drive. The two front wheels work together to steer the car, and all wheels provide the same traction and forward and backward motion to drive the car.

This resembles the Bicycle Steering model. Therefore, a bicycle steering controller is used for this LEGO car, utilizing two joints, virtual_rear_wheel_joint and virtual_front_wheel_joint, to control its movement.

For more information about the ROS2 Steering Controller, please see the documentation on [control.ros.org](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html) and [steering_controllers](https://control.ros.org/humble/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html#steering-controllers-library-userdoc). The second article provides detailed information on the controller's command and state interfaces, parameters, and subscribed and published topics. 

## Build
To build the project, we support Docker or a local Linux environment. 

### Docker
The Docker image provides a ROS2 environment with all dependencies. To build the Docker image, use this command: 

```bash
docker build -t lego_audi_etron .
```

Note: I could not build the Docker image on the Raspberry Pi due to limited compute resources for building the SimpleBLE C++ library. I had to build the image on my Linux dev machine, push it to the Docker Hub, and pull it onto the Pi. Therefore, I created a script for building and pushing the docker image: 
`./scripts/publish-docker.sh`. 

### Local
if not using Docker, follow these instructions to build the package on the Linux machine:

1. Build and install `SimpleBLE`, which provides Bluetooth support for the LEGO hub and motors.
* The source code for SimpleBLE is at: https://simpleble.readthedocs.io/en/latest/overview.html
* For the library's build and installation instructions, please see [Build_SimpleBLE.md](./Build_SimpleBLE.md)
2. build the audi_etron ROS2 package in the ROS workspace directory. 
   ```bash
   colcon build --packages-select=audi_etron
   source install/setup.bash 
   ```

## Quick Play:

Run the robot in the real world and view it in **RViz**. RViz is a 3D visualizer for the ROS framework, used to display sensor data (LiDAR, cameras), robot models (URDF), and system states in real-time.

As with the build, we support running the robot as a Docker container or in a local Linux environment.

### Docker

Turn on the LEGO Hub, and immediately start the "lego_audi_etron" docker container. The container's launch file loads and starts the robot hardware and controllers.

```bash
./scripts/start-robot.sh
```

Use the --pull flag to force a pull of the image from the Docker Hub repository if the image has been updated in the hub. If the image is private, use `docker login` to log into the Docker Hub first.

```bash
docker login
./scripts/start-robot.sh --pull
```

To inspect the Docker container's directory structure, we can use `docker run` to get an interactive shell in the container without starting the robot
```bash
docker run -it --rm  <image_name> /bin/bash
```

About Docker container's D-Bus and Bluetooth access:
- `--privileged`: Required to bypass AppArmor restrictions and access D-Bus/Bluetooth hardware
- `-v /var/run/dbus:/var/run/dbus`: Mounts the host's D-Bus socket so SimpleBLE can communicate with BlueZ
- SimpleBLE uses D-Bus to communicate with the BlueZ daemon for Bluetooth Low Energy operations

#### Visualization in RViz

To view the robot in RViz, I use my Linux dev machine. The `ROS_DOMAIN_ID` environment variable must be the same on the dev machine and in the container. This variable controls who can access the robot's published data and which set of ROS2 applications can interact with each other. If you cannot see the robot in RViz but the robot is running, please check that the `ROS_DOMAIN_ID` variable is set correctly. 

```bash
ros2 launch audi_etron view_robot_rviz2.launch.py
```

### Local

Turn on the LEGO Hub, and start the robot with RViz on the Linux machine. The launch file loads and starts the robot hardware, controllers and opens RViz.
```bash
ros2 launch audi_etron lego_audi_etron.launch.py remap_odometry_tf:=true
```

Note, to see only the robot description, use the following command. The "-d" option launches the robot in the debug log level.
```bash
ros2 launch audi_etron view_robot.launch.py -d
```

### Hardware Interfaces and Controllers
Commands to introspect the control system before driving the robot use the joystick

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

   We should see this:
   ```
   joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
   bicycle_steering_controller bicycle_steering_controller/BicycleSteeringController  active
   ```

### Send commands for the car to circling

If everything is fine, now we can send commands to drive the LEGO car. 

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

To use a joystick (gamepad) controlling the LEGO car, launch the `teleop_twist_joy` node.

Each gamepad has an enable button. Press it while twisting the joystick to control the robot. On the PS5, it is the "PS" button.  

```bash
# Install the ROS2 teleop-twist-joy package if have not already installed
sudo apt install ros-${ROS_DISTRO}-teleop-twist-joy

# launch the teleop-twist-joy 
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' publish_stamped_twist:=true
```

### Troubleshooting

For common issues (e.g. joystick timestamp errors when using Mac + Raspberry Pi) and time-sync steps, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

