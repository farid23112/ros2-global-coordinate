import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GlobalCoordinate(Node):
    """Waypoint controller that consumes /global_pose and publishes /input_ik."""

    def __init__(self):
        super().__init__("global_coordinate")

        self.pose = None
        self.initialized = False
        self.finished = False

        self.pose_sub = self.create_subscription(
            Pose2D, "/global_pose", self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/input_ik", 10)

        self.length = 4.0
        self.width = 2.0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.position_tolerance = 0.08
        self.angle_tolerance = 0.05

        self.local_points = [
            (self.length, 0.0),
            (self.length, self.width),
            (0.0, self.width),
            (0.0, 0.0),
        ]

        self.targets = []
        self.target_index = 0
        self.phase = "turn"
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Global Coordinate Controller dimulai")

    def pose_callback(self, msg: Pose2D):
        self.pose = msg
        if self.initialized:
            return

        x0, y0, theta0 = msg.x, msg.y, msg.theta
        cos_theta = math.cos(theta0)
        sin_theta = math.sin(theta0)

        for local_x, local_y in self.local_points:
            global_x = x0 + local_x * cos_theta - local_y * sin_theta
            global_y = y0 + local_x * sin_theta + local_y * cos_theta
            self.targets.append((global_x, global_y))

        self.initialized = True
        self.get_logger().info(
            f"Pose awal: x={x0:.2f}, y={y0:.2f}, "
            f"theta={math.degrees(theta0):.1f} deg"
        )

    def publish_velocity(self, linear: float, angular: float):
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)

    def control_loop(self):
        if self.pose is None or not self.initialized:
            return

        if self.finished:
            self.publish_velocity(0.0, 0.0)
            return

        x, y, theta = self.pose.x, self.pose.y, self.pose.theta
        target_x, target_y = self.targets[self.target_index]

        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - theta)

        if self.phase == "turn":
            if abs(angle_error) > self.angle_tolerance:
                angular = max(
                    -self.angular_speed,
                    min(self.angular_speed, 1.5 * angle_error),
                )
                self.publish_velocity(0.0, angular)
            else:
                self.phase = "move"
                self.publish_velocity(0.0, 0.0)
            return

        if self.phase == "move":
            if distance <= self.position_tolerance:
                self.publish_velocity(0.0, 0.0)
                self.get_logger().info(
                    f"Waypoint {self.target_index + 1} tercapai: "
                    f"x={x:.2f}, y={y:.2f}"
                )
                self.target_index += 1

                if self.target_index >= len(self.targets):
                    self.finished = True
                    self.get_logger().info("Lintasan persegi panjang selesai")
                else:
                    self.phase = "turn"
                return

            angular = max(
                -self.angular_speed,
                min(self.angular_speed, 1.5 * angle_error),
            )
            self.publish_velocity(self.linear_speed, angular)

        self.get_logger().info(
            f"Pose global: x={x:.2f} m, y={y:.2f} m, "
            f"theta={math.degrees(theta):.1f} deg",
            throttle_duration_sec=1.0,
        )

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCoordinate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
