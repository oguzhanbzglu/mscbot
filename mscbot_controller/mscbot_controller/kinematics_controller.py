#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from tf_transformations import quaternion_from_euler

class KinematicsController(Node):

    def __init__(self):
        super().__init__("kinematics_controller_node")
 
        # Wheel dynamic parameters
        self.R = 0.085
        self.L = 0.4

        # initial vehicle parameters - definetions to compute the linear and angular velocities of the robot based on the wheels' pose
        self.left_wheel_prev_pose_ = 0.0
        self.right_wheel_prev_pose_ = 0.0
        self.prev_time_ = self.get_clock().now()        #current moment in time

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.get_logger().info("Using wheel radius %f" % self.R)
        self.get_logger().info("Using wheel separation %f" % self.L)

        #Creating publisher and subscriber
        """ TwistStamped message 
        Header header
            uint32 seq
            time stamp
            string frame_id
        Twist twist
            geometry_msgs/Vector3 linear
                float64 x
                float64 y
                float64 z
            geometry_msgs/Vector3 angular
                float64 x
                float64 y
                float64 z
        """

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "kinematics_controller/cmd_vel", self.velCallback, 10)
        self.joint_state_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "kinematics_controller/odom", 10)

        #Kinematic mtrix
        self.speed_conversion_ = np.array([[self.R/2, self.R/2],
                                           [self.R/self.L, -self.R/self.L]])
        
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

        # initializng the odom msg
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0


    def velCallback(self, msg):
        """msg: it is the last message that pyblished from the topic!!"""
        # Implements the differential kinematic model
        # Given v and w, calculate the velocities of the wheels

        #invertinf speed conversation
        inverse_speed_conversion_ = np.linalg.inv(self.speed_conversion_)
        linear_x = msg.twist.linear.x
        angular_z = msg.twist.angular.z

        #convert the speedto an array
        robot_speed = np.array([[linear_x],
                                [angular_z]])
        #matrix multiply for the speeds of each wheel
        wheel_speed = np.matmul(inverse_speed_conversion_, robot_speed)
        
        #define the message to publish right and left wheels speed
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], 
                                wheel_speed[0, 0]]
        
        #publish msg
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
    
    def jointCallback(self, msg):
        # pose change over time
        dPoseRight = msg.position[0] - self.right_wheel_prev_pose_
        dPoseLeft = msg.position[1] - self.left_wheel_prev_pose_
        dTime = Time.from_msg(msg.header.stamp) - self.prev_time_
        
        #updating the pose of the wheels
        self.right_wheel_prev_pose_ = msg.position[0]
        self.left_wheel_prev_pose_ = msg.position[1]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # velocity of wheels based on its positions
        phiDotR = dPoseRight/ (dTime.nanoseconds / S_TO_NS)
        phiDotL = dPoseLeft / (dTime.nanoseconds  / S_TO_NS)

        linear_vel = (self.R/2)*phiDotR + (self.R/2)*phiDotL
        angular_vel = (self.R/self.L)*phiDotR - (self.R/self.L)*phiDotL

        d_s = (self.R/2)*dPoseRight + (self.R/2)*dPoseLeft                      #Traveled distance (x, y)
        d_theta = (self.R/self.L)*dPoseRight - (self.R/self.L)*dPoseLeft        #Traveled orientation (theta)

        #updated positions of the robot - Odometry
        self.theta_ += d_theta
        self.x_ += d_s * np.cos(self.theta_)
        self.y_ += d_s * np.sin(self.theta_)

        # odometry message
        #orientation
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        #time
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        #position
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        #velocity
        self.odom_msg_.twist.twist.linear.x = linear_vel
        self.odom_msg_.twist.twist.angular.z = angular_vel

        self.odom_pub_.publish(self.odom_msg_)

        # self.get_logger().info("Linear velocity: %f, Angular velocity: %f" % (linear_vel, angular_vel))
        # self.get_logger().info("x: %f, y: %f, theta: %f" % (self.x_, self.y_, self.theta_))

def main():
    rclpy.init()

    kinematics_controller = KinematicsController()
    rclpy.spin(kinematics_controller)
    
    kinematics_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()