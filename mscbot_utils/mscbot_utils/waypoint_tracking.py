#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistStamped
import math as m
import os
import matplotlib.pyplot as plt

class WaypointTracking(Node):
    def __init__(self):
        super().__init__('waypoint_tracking')

        # Publishers and subscribers
        self.odom_sub_ = self.create_subscription(Odometry, 'kinematics_controller/odom', self.odomCallback, 10)
        self.cmd_vel_pub_ = self.create_publisher(TwistStamped, 'kinematics_controller/cmd_vel', 10)
        self.model_state_sub_ = self.create_subscription(ModelStates, '/gazebo/model_states', self.modelStateCallback, 10)

        self.package_name = 'mscbot_utils'
        self.package_path = self.find_package_path(self.package_name)
        self.get_logger().info('The Waypoint Tracking node has been started!')
        self.get_logger().info('The package path is %s' % (self.package_path))

        # Get the waypoints
        self.path_directory = self.package_path + '/path/waypoint.txt'
        f = open(self.path_directory, 'r')
        self.waypoints = f.readlines()
        f.close()
        self.points = []

        for point in self.waypoints:
            value = point.split()
            x = float(value[0])
            y = float(value[1])
            self.points.append((x, y))

        self.distanceTolerance = 0.1
        self.index = 0
        self.x_goal = 0
        self.y_goal = 0

        self.max_lin_vel = 0.5
        self.max_ang_vel = 1.0

        # Store the robot's pose from odometry and Gazebo
        self.robot_pose = []
        self.gazebo_pose = []

        # PID controller for linear velocity
        self.kP = 0.025
        self.kI = 0.01
        self.kD = 0.2
        self.prev_error = 0.0
        self.integralErr = 0.0

        # PID controller for angular velocity
        self.kP_w = 1.5
        self.kI_w = 0.05
        self.kD_w = 0.1
        self.prev_error_w = 0.0
        self.integralErr_w = 0.0

    def find_package_path(self, pkg):
        for path in os.getenv('AMENT_PREFIX_PATH', '').split(os.pathsep):
            path_ = os.path.join(path, 'share', pkg)
            if os.path.isdir(path_):
                return os.path.abspath(os.path.join(path, '..', '..', 'src','mscbot', pkg))
            return None

    def odomCallback(self, msg):
        if self.index >= len(self.points):
            self.get_logger().info('All waypoints have been reached. Stopping the robot.')
            cmd_vel_msg = TwistStamped()
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = 0.0
            self.cmd_vel_pub_.publish(cmd_vel_msg)
            self.plot_position()  # Plot the position when all waypoints are reached
            return

        cmd_vel_msg = TwistStamped()
        
        # Robot's current pose
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        # Get yaw angle
        current_theta = 2 * m.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        # Goal pose
        self.x_goal = self.points[self.index][0]
        self.y_goal = self.points[self.index][1]

        # Position error
        d_x = self.x_goal - current_x
        d_y = self.y_goal - current_y

        theta_goal = m.atan2(d_y, d_x)
        d_theta = theta_goal - current_theta

        # Normalizing the heading angle between +pi, -pi
        d_theta = m.atan2(m.sin(d_theta), m.cos(d_theta))

        distance_to_goal = m.sqrt(d_x**2 + d_y**2)

        """PID Control for linear velocity"""
        error = distance_to_goal  # Error as distance
        self.integralErr += error * 0.001  # Assuming time step is 0.01
        self.integralErr = max(min(self.integralErr, 10), -10)  # Anti-windup
        derivative = error - self.prev_error
        self.prev_error = error
        pid_output = self.kP * error + self.kI * self.integralErr + self.kD * derivative
        linear_vel = max(min(pid_output, self.max_lin_vel), 0)  # No negative speeds


        """PID Control for angular velocity"""
        error_dtheta = d_theta  # Error as distance
        self.integralErr_w += error_dtheta * 0.001  # Assuming time step is 0.01
        self.integralErr_w = max(min(self.integralErr_w, 10), -10)  # Anti-windup
        derivative_w = error_dtheta - self.prev_error_w
        self.prev_error_w = error_dtheta
        pid_output_w = self.kP_w * error_dtheta + self.kI_w * self.integralErr_w + self.kD_w * derivative_w
        omega = max(min(pid_output_w, self.max_ang_vel), -self.max_ang_vel)  # Apply speed limit

        # d_theta = max(min(d_theta, self.max_ang_vel), -self.max_ang_vel)

        if distance_to_goal <= self.distanceTolerance:
            self.get_logger().info('The robot has reached the waypoint x: %f, y: %f' % (current_x, current_y))
            self.index += 1
            if self.index >= len(self.points):
                self.get_logger().info('The robot has reached the end')
                cmd_vel_msg.twist.linear.x = 0.0
                cmd_vel_msg.twist.angular.z = 0.0
                self.cmd_vel_pub_.publish(cmd_vel_msg)
            else:
                cmd_vel_msg.twist.linear.x = linear_vel
                cmd_vel_msg.twist.angular.z = omega
        else:
            cmd_vel_msg.twist.linear.x = linear_vel
            cmd_vel_msg.twist.angular.z = omega
        
        self.cmd_vel_pub_.publish(cmd_vel_msg)

        # Accumulating the coordinates
        self.robot_pose.append((current_x, current_y))

    def modelStateCallback(self, msg):
        model_name = 'mscbot'  # Replace with your model's name in Gazebo
        if model_name in msg.name:
            index = msg.name.index(model_name)
            model_pose = msg.pose[index]
            gazebo_x = model_pose.position.x
            gazebo_y = model_pose.position.y
            self.gazebo_pose.append((gazebo_x, gazebo_y))

    def plot_position(self):
        # Extract x and y coordinates for the robot's trajectory
        x_coords, y_coords = zip(*self.robot_pose) if self.robot_pose else ([], [])

        # Extract x and y coordinates for the Gazebo model's trajectory
        gazebo_x_coords, gazebo_y_coords = zip(*self.gazebo_pose) if self.gazebo_pose else ([], [])

        # Extract x and y coordinates for the waypoints, ignoring the third value
        waypoint_x = [float(point.split()[0]) for point in self.waypoints]
        waypoint_y = [float(point.split()[1]) for point in self.waypoints]

        # Plot the robot's trajectory from odometry
        plt.plot(x_coords, y_coords, label="Odometry Trajectory", color="blue")

        # Plot the robot's trajectory from Gazebo
        plt.plot(gazebo_x_coords, gazebo_y_coords, label="Gazebo Trajectory", color="green", linestyle="--")

        # Plot the waypoints and unite them with a line
        plt.plot(waypoint_x, waypoint_y, label="Waypoints", color="red", linestyle="--", marker="o")

        # Add labels and title
        plt.title('Robot Trajectory')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.legend()
        plt.grid(True)
        plt.show()
        

def main(args=None):
    rclpy.init(args=args)
    waypoint_tracking = WaypointTracking()
    rclpy.spin(waypoint_tracking)
    waypoint_tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
