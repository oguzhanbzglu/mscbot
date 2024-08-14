#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        #creating subscriber
        self.odom_sub_ = self.create_subscription(Odometry, '/kinematics_controller/odom', self.odomCallback, 10)
        self.path_pub_ = self.create_publisher(Path, 'kinematics_controller/trajectory', 10)

        
        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = 'base_link'

        self.get_logger().info("The Path Publisher node has been started!")

        
    def odomCallback(self, msg):
        pose_msg_ = PoseStamped()
        pose_msg_.header.frame_id = 'orientation_link'
        pose_msg_.header.stamp = self.get_clock().now().to_msg()
        pose_msg_.pose.position = msg.pose.pose.position
        pose_msg_.pose.orientation = msg.pose.pose.orientation
        # pose_msg_.pose.position.x = msg.pose.pose.position.x
        # pose_msg_.pose.position.y = msg.pose.pose.position.y
        # pose_msg_.pose.position.z = msg.pose.pose.position.z
        # pose_msg_.pose.orientation.x = msg.pose.pose.orientation.x
        # pose_msg_.pose.orientation.y = msg.pose.pose.orientation.y
        # pose_msg_.pose.orientation.z = msg.pose.pose.orientation.z
        # pose_msg_.pose.orientation.w = msg.pose.pose.orientation.w
        
        self.path_msg_.header.stamp = self.get_clock().now().to_msg()
        self.path_msg_.poses.append(pose_msg_)
        self.path_pub_.publish(self.path_msg_)
        

def main(args=None):
    rclpy.init(args=args)
    trajector_publisher = TrajectoryPublisher()
    rclpy.spin(trajector_publisher)
    trajector_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()