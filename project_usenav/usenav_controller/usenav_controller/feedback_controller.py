#!/usr/bin/env python3


# --- std ---
import math
import argparse

from typing import Tuple

# --- ros2 ---
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# --- ros2-interfaces ---
from std_msgs.msg import Bool

# from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist


# --- CONSTANTS ---
START: Tuple = (0.0, 0.0)  # start position (X, Y)
GOAL: Tuple = (2.74446, -6.14621)  # goal position (X, Y)
# GOAL: Tuple = (10, 10)  # goal position (X, Y)
GOAL_TOLERANCE = 0.20  # tolerance for reaching goal
Kp: Tuple = (9.4, 3.0)  # P coefficient for distance and angle
SPEED_BOUNDS: Tuple = (-5.0, 5.0)


def clamp(a: float, bounds: tuple) -> float:
    return float(max(bounds[0], min(a, bounds[1])))


def wrap_angle(a: float) -> float:
    # range -pi to pi
    return math.atan2(math.sin(a), math.cos(a))


def quaternion2theta(
    x: float, y: float, z: float, w: float, degrees: bool = False
) -> float:
    """calculates theta (yaw, aka angle from x axis in xy plane) from quaternion

    Args:
        x (float): _description_
        y (float): _description_
        z (float): _description_
        w (float): _description_

    Returns:
        float: theta in radians
    """
    theta_radians = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    if degrees:
        return math.degrees(theta_radians)

    return theta_radians


class FeedbackController(Node):
    def __init__(self, node_name: str = "feedback_control", x=10.00, y=10.00) -> None:
        super().__init__(node_name)
        global GOAL
        GOAL = (x, y)

        self.odometry_sub = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )
        self.get_logger().info("topic: /odom subscribed.")

        self.wheel_velocities_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
        self.get_logger().info(
            "Publishing wheel-velocities commands on the topic: /velocity_controller/commands subscribed."
        )

        self.goal_status_pub = self.create_publisher(Bool, "/goal_reached", 10)
        self.get_logger().info("Publishing Goal status on the topic: /goal_reached")

    def listener_callback(self, msg: Odometry) -> None:
        goal_reached = Bool()
        goal_reached.data = False

        # current position (x, y) in meters
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # quaternion form (x, y, z, w)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # radians
        current_theta = quaternion2theta(qx, qy, qz, qw, degrees=False)

        # self.get_logger().info(f"Odom: ({current_x}, {current_y}) at {current_theta} ---------------------")

        error = (GOAL[0] - current_x, GOAL[1] - current_y)
        error_magnitude = (error[0] ** 2 + error[1] ** 2) ** (1 / 2)
        angle_goal = math.atan2(
            error[1],  # vy
            error[0],  # vx
        )  # radians
        # self.get_logger().info(f"angle to goal: {math.degrees(angle_goal)}")

        # TODO double check units and domain
        orientation_error = wrap_angle(angle_goal - current_theta)
        # self.get_logger().info(f"Error: {error}, {math.degrees(orientation_error)}")

        desired_v = (
            Kp[0] * error[0],  # vx
            Kp[0] * error[1],  # vy
        )
        V = (desired_v[0] ** 2 + desired_v[1] ** 2) ** (1 / 2)
        wheel_linear_speed = clamp(V / 2, SPEED_BOUNDS) # This is linear.x

        # self.get_logger().info(f"wheel vel: {V/2}")
        # self.get_logger().info(f"steering angle: {alpha}")
        
        wheel_angular_speed = clamp(Kp[1] * orientation_error, SPEED_BOUNDS)

        wheel_velocities = Twist()

        # check reached goal?
        # if False:
        if error_magnitude < GOAL_TOLERANCE:
            self.get_logger().info("Goal reached!")
            goal_reached.data = True
            # wheel_velocities.data = [0.0, 0.0]
            wheel_velocities.linear.x =0.0
            wheel_velocities.angular.z =0.0

            self.wheel_velocities_pub.publish(wheel_velocities)
            self.goal_status_pub.publish(goal_reached)

            self.destroy_subscription(self.odometry_sub)
            self.odometry_sub = None
        else:
            self.get_logger().info(
                f"wheel vel (Linear.x): {wheel_linear_speed:.4f}, angular vel (Angular.z): {wheel_angular_speed:.4f}"
            )
            
            # Publish linear and angular speed as a Twist message
            wheel_velocities.linear.x = wheel_linear_speed
            wheel_velocities.angular.z = wheel_angular_speed


        # Publish
        self.wheel_velocities_pub.publish(wheel_velocities)
        self.goal_status_pub.publish(goal_reached)



def main(args=None):
    # Parse command-line argument for KITTI Data
    parser = argparse.ArgumentParser(description="Coordinates of the goal.")
    parser.add_argument("--x", type=float, default=10.00, help="x-coordinate")
    parser.add_argument("--y", type=float, default=10.00, help="y-coordinate")
    user_args = parser.parse_args()

    # x = user_args.x
    # y = user_args.y

    x,y = GOAL
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()

    node_name: str = "feedback_control"
    node = FeedbackController(node_name, x, y)

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard Exit, shutting down...")
    finally:
        # Destroy nodes that might not have self-destroyed
        if rclpy.ok():
            try:
                executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass  # Node may have already destroyed itself

        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
