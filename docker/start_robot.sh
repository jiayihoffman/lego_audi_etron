#!/bin/bash
ros2 launch my_bot robot.launch.py &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &