#!/bin/bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/ros/lego_ws/install/setup.bash

# Start ROS2 services in background
echo "Starting rosbridge_server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

echo "Starting robot launch (without GUI for Docker)..."
ros2 launch audi_etron carlikebot.launch.py gui:=false remap_odometry_tf:=true &
ROBOT_PID=$!

echo "ROS2 services started. PIDs: rosbridge=$ROSBRIDGE_PID, robot=$ROBOT_PID"

# Keep script running and monitor processes
# If a process exits, log it but keep the script running
while true; do
    sleep 10
    if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
        echo "WARNING: rosbridge_server (PID $ROSBRIDGE_PID) has exited"
    fi
    if ! kill -0 $ROBOT_PID 2>/dev/null; then
        echo "WARNING: robot (PID $ROBOT_PID) has exited"
    fi    
done
