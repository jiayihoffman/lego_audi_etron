#!/bin/bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/ros/lego_ws/install/setup.bash
export ROS_DOMAIN_ID=4

echo "Starting robot launch (without GUI for Docker)..."
ros2 launch audi_etron lego_audi_etron.launch.py gui:=false remap_odometry_tf:=true &
ROBOT_PID=$!

echo "Robot launch started. PID: robot=$ROBOT_PID"
echo "Press Ctrl+C to stop gracefully..."

# Signal handler function
cleanup() {
    echo ""
    echo "Received shutdown signal. Stopping robot launch..."
    
    # Forward SIGINT to child process
    if kill -0 $ROBOT_PID 2>/dev/null; then
        echo "Stopping robot launch (PID $ROBOT_PID)..."
        kill -INT $ROBOT_PID 2>/dev/null
    fi
    
    # Wait for process to terminate (with timeout)
    wait_timeout=10
    for i in $(seq 1 $wait_timeout); do
        if ! kill -0 $ROBOT_PID 2>/dev/null; then
            echo "Robot process stopped gracefully."
            exit 0
        fi
        sleep 1
    done
    
    # Force kill if still running
    if kill -0 $ROBOT_PID 2>/dev/null; then
        echo "Process did not stop within timeout. Force killing..."
        kill -KILL $ROBOT_PID 2>/dev/null
    fi
    
    exit 0
}

# Register signal handlers
trap cleanup SIGINT SIGTERM

# Keep script running and monitor process
while true; do
    sleep 1
    if ! kill -0 $ROBOT_PID 2>/dev/null; then
        echo "WARNING: robot (PID $ROBOT_PID) has exited"
        exit 0
    fi    
done