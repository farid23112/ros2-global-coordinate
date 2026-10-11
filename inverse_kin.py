import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class InverseKinematics(Node):
    """Convert body velocity commands into differential-drive wheel speeds."""

    def __init__(self):
        super().__init__("inverse_kin")

        self.wheel_radius = 0.10
        self.wheel_separation = 0.45

        self.left_pub = self.create_publisher(
            Float64, "/left_wheel/command", 10
        )
        self.right_pub = self.create_publisher(
            Float64, "/right_wheel/command", 10
        )
        # Some Gazebo robot setups also use /cmd_vel as the drive interface.
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            Twist, "/input_ik", self.command_callback, 10
        )
        self.get_logger().info("Inverse Kinematics aktif: /input_ik")

    def command_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = (
            linear - angular * self.wheel_separation / 2.0
        ) / self.wheel_radius
        right_speed = (
            linear + angular * self.wheel_separation / 2.0
        ) / self.wheel_radius

        left_msg = Float64()
        left_msg.data = float(left_speed)
        right_msg = Float64()
        right_msg.data = float(right_speed)

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # Forward Twist for models whose Gazebo plugin subscribes to /cmd_vel.
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
