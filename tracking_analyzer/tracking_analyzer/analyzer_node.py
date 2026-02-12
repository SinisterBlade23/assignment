import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from turtlesim.msg import Pose
import math
import matplotlib.pyplot as plt


class TrackingAnalyzer(Node):

    def __init__(self):
        super().__init__('tracking_analyzer')

        self.reference_path = []
        self.errors = []
        self.rms_values = []

        self.last_pose_time = None
        self.plot_generated = False

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer to check when robot stops
        self.create_timer(0.5, self.check_completion)

        self.get_logger().info("Tracking Analyzer Started")

    def trajectory_callback(self, msg):
        self.reference_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        self.errors = []
        self.rms_values = []
        self.plot_generated = False

        self.get_logger().info("Reference path received.")

    def pose_callback(self, msg):

        if not self.reference_path:
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

        self.errors.append(min_dist)

        # Compute cumulative RMS
        squared_sum = sum(e**2 for e in self.errors)
        rms = math.sqrt(squared_sum / len(self.errors))
        self.rms_values.append(rms)

    def check_completion(self):
        if self.plot_generated:
            return

        if self.last_pose_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pose_time).nanoseconds / 1e9

        # If no pose updates for 1 second → robot stopped
        if dt > 1.0 and self.rms_values:
            self.get_logger().info("Trajectory complete. Generating RMS plot...")
            self.plot_results()
            self.plot_generated = True

    def plot_results(self):
        plt.figure()
        plt.plot(self.rms_values)
        plt.xlabel("Time Step")
        plt.ylabel("Cumulative RMS Error (m)")
        plt.title("RMS Tracking Error Over Time")
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
