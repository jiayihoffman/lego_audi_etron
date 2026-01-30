#!/usr/bin/env python3
#
# Helper script: straighten the steering wheel.
# By default straightens from right turn (turns left to center).
# Use --from-left to straighten from left turn (turns right to center).
#
# Usage:
#   ./scripts/straighten_wheel.py              # straighten from right
#   ./scripts/straighten_wheel.py --from-left  # straighten from left
#

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


def main():
    # Parse args
    from_left = "--from-left" in sys.argv
    
    rclpy.init()
    node = Node("straighten_wheel")
    pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)
    
    # Straighten parameters
    # Need small linear.x for controller to compute steering (can't be zero)
    linear_x = 0.15 # small forward velocity to enable steering
    angular_z = -0.1 if from_left else 0.1  # opposite of the turn direction
    duration = 0.5  # seconds
    rate_hz = 50
    sleep_duration = 1.0 / rate_hz
    
    direction = "left" if from_left else "right"
    node.get_logger().info(f"Straightening wheel from {direction} turn (slow forward)...")
    
    start_time = time.time()
    try:
        while rclpy.ok() and (time.time() - start_time) < duration:
            now = node.get_clock().now()
            msg = TwistStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = ""
            msg.twist.linear.x = linear_x
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = angular_z
            pub.publish(msg)
            time.sleep(sleep_duration)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        now = node.get_clock().now()
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ""
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        pub.publish(msg)
        
        node.get_logger().info("Done.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
