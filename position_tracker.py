import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D


class PositionTracker(Node):
    """Estimasi pose robot dari perubahan sudut roda (dead reckoning)."""

    def __init__(self):
        super().__init__("position_tracker")

        self.wheel_radius = 0.10
        self.wheel_separation = 0.45

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.previous_left = None
        self.previous_right = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.pose_pub = self.create_publisher(
            Pose2D,
            "/global_pose",
            10,
        )

        self.timer = self.create_timer(0.5, self.display_position)
        self.get_logger().info("Position Tracker XY aktif.")

    def joint_state_callback(self, msg: JointState):
        try:
            left_index = msg.name.index("base_left_wheel_joint")
            right_index = msg.name.index("base_right_wheel_joint")
        except ValueError:
            return

        if left_index >= len(msg.position) or right_index >= len(msg.position):
            return

        left_position = msg.position[left_index]
        right_position = msg.position[right_index]

        if self.previous_left is None:
            self.previous_left = left_position
            self.previous_right = right_position
            return

        delta_left = left_position - self.previous_left
        delta_right = right_position - self.previous_right

        self.previous_left = left_position
        self.previous_right = right_position

        distance_left = self.wheel_radius * delta_left
        distance_right = self.wheel_radius * delta_right

        distance = (distance_left + distance_right) / 2.0
        delta_theta = (
            distance_right - distance_left
        ) / self.wheel_separation

        theta_middle = self.theta + delta_theta / 2.0
        self.x += distance * math.cos(theta_middle)
        self.y += distance * math.sin(theta_middle)
        self.theta = math.atan2(
            math.sin(self.theta + delta_theta),
            math.cos(self.theta + delta_theta),
        )

    def display_position(self):
        if self.previous_left is None:
            self.get_logger().info(
                "Menunggu data /joint_states...",
                throttle_duration_sec=2.0,
            )
            return

        pose = Pose2D()
        pose.x = float(self.x)
        pose.y = float(self.y)
        pose.theta = float(self.theta)
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"POSISI ROBOT -> X: {self.x:.3f} m | "
            f"Y: {self.y:.3f} m | "
            f"THETA: {math.degrees(self.theta):.1f} deg",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PositionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
