#!/usr/bin/env python3

# Smoothing of trajectory with cubic splines

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.interpolate import CubicSpline


class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother')
        
        # Subscriber to waypoints
        self.waypoint_sub = self.create_subscription(
            PoseArray,
            '/waypoints',
            self.waypoint_callback,
            10
        )
            
        
        self.trajectory_pub = self.create_publisher(Path, '/trajectory', 10)
        
        # Parameters
        self.num_points = 200  # Number of points to sample from spline
        self.velocity = 1.0     # Constant velocity 
        
        self.get_logger().info('Path Smoother Node started, waiting for waypoints')
        
    def waypoint_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.poses)} waypoints')
        
        
        waypoints = [(pose.position.x, pose.position.y) for pose in msg.poses]# Extract x, y coordinates
        
        if len(waypoints) < 2:
            self.get_logger().warn('Need at least 2 waypoints for smoothing')
            return
            
        smooth_trajectory = self.smooth_path(waypoints)
        
        # Publish trajectory
        self.publish_trajectory(smooth_trajectory)
        
    def smooth_path(self, waypoints):

        x_points = np.array([w[0] for w in waypoints])
        y_points = np.array([w[1] for w in waypoints])
        
        # Create parameter t (0 to n-1 for n waypoints)
        t_waypoints = np.arange(len(waypoints))
        
        
        cs_x = CubicSpline(t_waypoints, x_points) # Create cubic splines for x(t) and y(t)
        cs_y = CubicSpline(t_waypoints, y_points)
        
        
        t_smooth = np.linspace(0, len(waypoints) - 1, self.num_points) # Sample the spline at high resolution
        x_smooth = cs_x(t_smooth)
        y_smooth = cs_y(t_smooth)
        
        
        distances = np.zeros(len(x_smooth)) # Calculate cumulative distance for time parameterization
        for i in range(1, len(x_smooth)):
            dx = x_smooth[i] - x_smooth[i-1]
            dy = y_smooth[i] - y_smooth[i-1]
            distances[i] = distances[i-1] + np.sqrt(dx**2 + dy**2)
        
        
        times = distances / self.velocity # Assign timestamps based on constant velocity
        
        trajectory = list(zip(x_smooth, y_smooth, times)) # Combine into trajectory
        
        total_distance = distances[-1]
        total_time = times[-1]
        
        self.get_logger().info(
            f'Generated smooth trajectory: {len(trajectory)} points, '
            f'{total_distance:.2f}m, {total_time:.2f}s'
        )
        
        return trajectory
        
    def publish_trajectory(self, trajectory):
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, t in trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        self.trajectory_pub.publish(path_msg)
        self.get_logger().info('Smooth trajectory published')


def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()