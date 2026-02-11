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
        self.start_time = None

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.get_logger().info("Tracking Analyzer Started")

    def trajectory_callback(self, msg):
        self.reference_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        self.errors = []
        self.rms_values = []
        self.start_time = self.get_clock().now()

        self.get_logger().info("Reference path received.")

    def pose_callback(self, msg):

        if not self.reference_path:
            return

        ax = msg.x
        ay = msg.y

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

    def plot_results(self):
        if not self.rms_values:
            return

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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.plot_results()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
