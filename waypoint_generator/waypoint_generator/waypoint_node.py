#!/usr/bin/env python3

# Waypoint Generator Node Generates 5 random waypoints 
# within turtlesim bounds and publishes them periodically.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import random

class WaypointGeneratorNode(Node):
    def __init__(self):
        super().__init__('waypoint_generator')
        
        self.waypoint_pub = self.create_publisher(PoseArray, '/waypoints', 10)
        
        self.min_coord = 1.0 # Turtlesim bounds (0-11 in both x and y)
        self.max_coord = 10.0
        
        
        self.stored_waypoints = self._create_random_waypoints()
        
        # Create a timer to publish them every 1.0 second
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        self.get_logger().info('Node started, waiting for publish')

    def _create_random_waypoints(self):
        waypoints = PoseArray()
        waypoints.header.frame_id = "world"
        waypoints.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(5): # Generate 5 random points
            pose = Pose()
            pose.position.x = random.uniform(self.min_coord, self.max_coord)
            pose.position.y = random.uniform(self.min_coord, self.max_coord)
            pose.position.z = 0.0
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            waypoints.poses.append(pose)
            
        # Log them once so we know what they are
        self.get_logger().info('Generated Waypoints (Static):')
        for i, pose in enumerate(waypoints.poses):
            self.get_logger().info(
                f'  Point {i+1}: ({pose.position.x:.2f}, {pose.position.y:.2f})'
            )
            
        return waypoints
        
    def publish_waypoints(self):
        #Timer callback to publish the stored waypoints
        self.stored_waypoints.header.stamp = self.get_clock().now().to_msg()
        
        self.waypoint_pub.publish(self.stored_waypoints)
        self.get_logger().info('Published waypoints...', throttle_duration_sec=2.0)

        self.timer.cancel()
        
        print("\n" + "="*40)
        print("  TURTLE MISSION PLAN (5 GOALS) ")
        print("="*40)
        
        for i, pose in enumerate(self.stored_waypoints.poses):
            print(f"  GOAL {i+1}: X = {pose.position.x:.2f}, Y = {pose.position.y:.2f}")
            
        print("="*40 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGeneratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()