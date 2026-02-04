#!/usr/bin/env python3
# Copyright (c) 2026 Jiayi Hoffman.
#
# LEGO car demo.
# Publishes TwistStamped to /cmd_vel at a fixed rate with fresh timestamp on
# every message (no timeout).
#
# Usage (after sourcing ROS2):
#   ./scripts/demo_drive.py
#
# Environment variables:
#   CMD_VEL_TOPIC - override topic (default /cmd_vel)
#

import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


# Dance parameters — tune to fit your room.
LINEAR_X = 1.0
LINEAR_X_SLOW = 0.8
ANGULAR_Z = 0.65
D_LONG = 3.0
D_MED = 1.5
D_SHORT = 1.0

LINEAR_X_STRAIGHTEN = 0.15  # Small velocity for straightening (controller needs non-zero linear.x)
ANGULAR_Z_STRAIGHTEN = 0.1
D_STRAIGHTEN = 0.5  # Short move to straighten wheel after turns

PUB_RATE = 50  # Hz — match controller; fresh stamp every message


def get_dance_sequence():
    """(linear_x, angular_z, duration_sec, description) for each move."""
    return [
        (LINEAR_X, 0.0, D_MED, "Forward"),
        (LINEAR_X_SLOW, ANGULAR_Z, D_SHORT, "Turn left"),
        (-LINEAR_X_SLOW, ANGULAR_Z, D_MED, "Back left"),
        (LINEAR_X_SLOW, -ANGULAR_Z, D_LONG, "Turn right"),
        (LINEAR_X_STRAIGHTEN, ANGULAR_Z_STRAIGHTEN, D_STRAIGHTEN, "Straighten wheel"),
        (LINEAR_X, 0.0, D_SHORT, "Forward"),
        (LINEAR_X_SLOW, -ANGULAR_Z, D_LONG, "Turn right"),
        (LINEAR_X_STRAIGHTEN, ANGULAR_Z_STRAIGHTEN, D_STRAIGHTEN, "Straighten wheel"),
        (LINEAR_X, 0.0, D_SHORT, "Forward"),
        (LINEAR_X_SLOW, -ANGULAR_Z, D_LONG, "Turn right"),
        (LINEAR_X_STRAIGHTEN, ANGULAR_Z_STRAIGHTEN, D_STRAIGHTEN, "Straighten wheel"),
        (LINEAR_X, 0.0, D_SHORT, "Forward"),
    ]


def main():
    rclpy.init()
    node = Node("demo_drive")
    default_topic = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
    topic = node.declare_parameter("cmd_vel_topic", default_topic).value
    pub = node.create_publisher(TwistStamped, topic, 10)
    
    sleep_duration = 1.0 / PUB_RATE  # seconds between publishes

    moves = get_dance_sequence()
    move_index = 0
    move_start_time = time.time()

    node.get_logger().info(
        f"Demo: {len(moves)} moves, {PUB_RATE} Hz, topic {topic}"
    )
    
    # Log first move
    if moves:
        node.get_logger().info(f"[1/{len(moves)}] {moves[0][3]}")

    try:
        while rclpy.ok() and move_index < len(moves):
            now = node.get_clock().now()
            msg = TwistStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = ""

            lx, az, duration, description = moves[move_index]
            msg.twist.linear.x = float(lx)
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = float(az)

            pub.publish(msg)

            elapsed = time.time() - move_start_time
            if elapsed >= duration:
                move_index += 1
                move_start_time = time.time()
                if move_index < len(moves):
                    node.get_logger().info(f"[{move_index + 1}/{len(moves)}] {moves[move_index][3]}")

            time.sleep(sleep_duration)

        # Stop: publish zero with fresh stamp for a short time.
        node.get_logger().info("Stopping...")
        stop_end = time.time() + 0.5
        while rclpy.ok() and time.time() < stop_end:
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
            time.sleep(sleep_duration)

        node.get_logger().info("Demo finished.")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
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
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
