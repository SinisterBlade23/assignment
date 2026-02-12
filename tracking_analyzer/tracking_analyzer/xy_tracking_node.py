import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from turtlesim.msg import Pose
import matplotlib.pyplot as plt


class XYTrackingAnalyzer(Node):

    def __init__(self):
        super().__init__('xy_tracking_analyzer')

        self.reference_x = []
        self.reference_y = []

        self.actual_x = []
        self.actual_y = []

        self.last_pose_time = None
        self.plot_generated = False

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer to detect when turtle stops
        self.create_timer(0.5, self.check_completion)

        self.get_logger().info("XY Tracking Analyzer Started")

    def trajectory_callback(self, msg):
        self.reference_x = [pose.pose.position.x for pose in msg.poses]
        self.reference_y = [pose.pose.position.y for pose in msg.poses]

        self.actual_x = []
        self.actual_y = []
        self.plot_generated = False

        self.get_logger().info("Reference trajectory received")

    def pose_callback(self, msg):
        self.actual_x.append(msg.x)
        self.actual_y.append(msg.y)

        self.last_pose_time = self.get_clock().now()

    def check_completion(self):
        if self.plot_generated:
            return

        if self.last_pose_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pose_time).nanoseconds / 1e9

        # If no pose updates for 1 second → robot stopped
        if dt > 1.0 and self.actual_x:
            self.get_logger().info("Trajectory complete. Generating XY plot...")
            self.plot_results()
            self.plot_generated = True

    def plot_results(self):
        plt.figure()
        plt.plot(self.reference_x, self.reference_y, label="Reference Path", linewidth=2)
        plt.plot(self.actual_x, self.actual_y, "--", label="Actual Path", linewidth=2)

        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Trajectory Tracking Performance")
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = XYTrackingAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
