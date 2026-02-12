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
        self.last_pose_time = None
        self.plot_generated = False

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer to detect completion
        self.create_timer(0.5, self.check_completion)

        self.get_logger().info("Instantaneous Error Analyzer Started")

    def trajectory_callback(self, msg):
        self.reference_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        self.errors = []
        self.times = []
        self.start_time = self.get_clock().now()
        self.plot_generated = False

        self.get_logger().info("Reference path received.")

    def pose_callback(self, msg):

        if not self.reference_path or self.start_time is None:
            return

        ax = msg.x
        ay = msg.y

        self.last_pose_time = self.get_clock().now()

        # Compute cross-track error
        min_dist = float('inf')
        for rx, ry in self.reference_path:
            dist = math.sqrt((ax - rx)**2 + (ay - ry)**2)
            if dist < min_dist:
                min_dist = dist

        # Time since start
        dt = (self.last_pose_time - self.start_time).nanoseconds / 1e9

        self.times.append(dt)
        self.errors.append(min_dist)

    def check_completion(self):
        if self.plot_generated:
            return

        if self.last_pose_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pose_time).nanoseconds / 1e9

        # If no pose updates for 1 second → robot stopped
        if dt > 1.0 and self.errors:
            self.get_logger().info("Trajectory complete. Generating Instant Error plot...")
            self.plot_results()
            self.plot_generated = True

    def plot_results(self):
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
