import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from turtlesim.msg import Pose
import math
import matplotlib.pyplot as plt


class InstantErrorAnalyzer(Node):

    def __init__(self):
        super().__init__('instant_error_analyzer')

        self.reference_path = []
        self.errors = []
        self.times = []
        self.start_time = None

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.get_logger().info("Instantaneous Error Analyzer Started")

    def trajectory_callback(self, msg):
        self.reference_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        self.errors = []
        self.times = []
        self.start_time = self.get_clock().now()

        self.get_logger().info("Reference path received.")

    def pose_callback(self, msg):

        if not self.reference_path or self.start_time is None:
            return

        # Current robot position
        ax = msg.x
        ay = msg.y

        # Compute cross-track error
        min_dist = float('inf')
        for rx, ry in self.reference_path:
            dist = math.sqrt((ax - rx)**2 + (ay - ry)**2)
            if dist < min_dist:
                min_dist = dist

        # Compute time since start (seconds)
        current_time = self.get_clock().now()
        dt = (current_time - self.start_time).nanoseconds / 1e9

        self.times.append(dt)
        self.errors.append(min_dist)

    def plot_results(self):
        if not self.errors:
            return

        plt.figure()
        plt.plot(self.times, self.errors)
        plt.xlabel("Time (s)")
        plt.ylabel("Instantaneous Cross-Track Error (m)")
        plt.title("Instantaneous Tracking Error Over Time")
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = InstantErrorAnalyzer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.plot_results()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
