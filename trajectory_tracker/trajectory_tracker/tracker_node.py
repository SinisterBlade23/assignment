#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TrajectoryTrackerNode(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        self.trajectory_sub = self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.kp_angular = 4.0 # Proportional gain
        self.ki_angular = 1.4 # Integral gain
        self.kd_angular = 0.0 # Derivative gain
        
        self.k_linear = 1.0      
        self.max_linear_vel = 2.0
        self.max_angular_vel = 2.0
        
        self.prev_angular_error = 0.0
        self.integral_angular_error = 0.0
        self.last_time = self.get_clock().now()
        
        self.goal_tolerance = 0.1 
        self.trajectory = None
        self.current_pose = None
        self.target_index = 0
        self.trajectory_complete = False
        
        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz control loop
        self.get_logger().info('PID Tracker initialized')

    def trajectory_callback(self, msg):
        self.trajectory = msg.poses
        self.target_index = 0
        self.trajectory_complete = False
        self.prev_angular_error = 0.0
        self.integral_angular_error = 0.0
        self.last_time = self.get_clock().now()
        self.get_logger().info(f'Received trajectory with {len(self.trajectory)} points')
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def control_loop(self):
        if self.trajectory is None or self.current_pose is None:
            return
            
        if self.trajectory_complete:
            self.publish_velocity(0.0, 0.0)
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 # Time delta in seconds
        self.last_time = current_time
        
        if dt <= 0:
            return

        target = self.trajectory[self.target_index]
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.goal_tolerance:
            self.target_index += 1
            
            if self.target_index % 50 == 0:
                goal_num = (self.target_index // 50) + 1
                self.get_logger().info(f"Reached waypoint {goal_num} approximation")

            if self.target_index >= len(self.trajectory):
                self.trajectory_complete = True
                self.get_logger().info('Trajectory complete')
                self.publish_velocity(0.0, 0.0)
                return
            else:
                target = self.trajectory[self.target_index]
                target_x = target.pose.position.x
                target_y = target.pose.position.y
                dx = target_x - self.current_pose.x
                dy = target_y - self.current_pose.y
                distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_pose.theta)
        
        p_term = self.kp_angular * angle_error
        self.integral_angular_error += angle_error * dt
        self.integral_angular_error = max(min(self.integral_angular_error, 1.0), -1.0) # Anti-windup
        i_term = self.ki_angular * self.integral_angular_error
        
        derivative = (angle_error - self.prev_angular_error) / dt
        d_term = self.kd_angular * derivative
        self.prev_angular_error = angle_error
        
        angular_vel = p_term + i_term + d_term

        is_last_point = (self.target_index == len(self.trajectory) - 1)
        if is_last_point:
            linear_vel = self.k_linear * distance
        else:
            linear_vel = self.max_linear_vel * (1.0 - min(abs(angle_error), 1.0)) # Scale speed based on turn error
        
        linear_vel = max(0.0, min(linear_vel, self.max_linear_vel))
        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
        
        self.publish_velocity(linear_vel, angular_vel)
        
    def publish_velocity(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
        
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()