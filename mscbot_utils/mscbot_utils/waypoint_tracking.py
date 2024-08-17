#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math as m
import os

class  WaypointTracking(Node):
    def __init__(self):
        super().__init__('waypoint_tracking')

        # publishers and subscribers
        self.odom_sub_ = self.create_subscription(Odometry, 'kinematics_controller/odom', self.odomCallback, 10)
        self.cmd_vel_pub_ = self.create_publisher(TwistStamped, 'kinematics_controller/cmd_vel', 10)
        self.package_name = 'mscbot_utils'
        self.package_path = self.find_package_path(self.package_name)
        self.get_logger().info('The Waypoint Tracking node has been started!')
        self.get_logger().info('The package paht is %s: ' % (self.package_path))

        #get the waypoints
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


        self.distanceTolerance = 0.5
        self.index = 0
        self.x_goal = 0
        self.y_goal = 0

        self.max_lin_vel = 1.0
        self.max_ang_vel = 1.0

        #PID parameters for linear velocity
        self.max_lin_vel = 1.0
        self.kp_v = 1.0
        self.ki_v = 0.1
        self.kd_v = 0.5

        self.iTerm_v = 0.0
        self.dTerm_v = 0.0
        self.previous_distance_error = 0.0

        #PID parameters for omega
        self.kp_w = 2.0
        self.ki_w = 0.0
        self.kd_w = 0.1

        self.iTerm_w = 0.0
        self.dTerm_w = 0.0
        self.previous_dTheta = 0.0

    def find_package_path(self, pkg):
        # Search through the paths in AMENT_PREFIX_PATH to find the package
        for path in os.getenv('AMENT_PREFIX_PATH', '').split(os.pathsep):
            path_ = os.path.join(path, 'share', pkg)
            if os.path.isdir(path_):
                return os.path.abspath(os.path.join(path, '..', '..', 'src','mscbot', pkg))
            return None
    
    def odomCallback(self, msg):

        cmd_vel_msg = TwistStamped()
        
        #robot's current pose
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        #Get yaw angle
        current_theta = 2 * m.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        #goal pose
        self.x_goal = self.points[self.index][0]
        self.y_goal = self.points[self.index][1]

        #position error
        d_x = self.x_goal - current_x
        d_y = self.y_goal - current_y

        theta_goal = m.atan2(d_y, d_x)
        d_theta = theta_goal - current_theta

        #normalizing the heading angle between +pi, -pi
        d_theta = m.atan2(m.sin(d_theta), m.cos(d_theta))

        distance_to_goal = m.sqrt(d_x**2 + d_y**2)
        # print("Distance to goal: ",distance_to_goal)

        # PID for Linear velocity
        self.iTerm_v += distance_to_goal
        self.dTerm_v = distance_to_goal - self.previous_distance_error
        linear_velocity = (
                self.kp_v * distance_to_goal   +
                self.ki_v * self.iTerm_v       +
                self.kd_v * self.dTerm_v
            )
        self.previous_distance_error = distance_to_goal
        linear_velocity = max(min(linear_velocity, self.max_lin_vel), -self.max_lin_vel)            # Apply a speed limit to prevent overshooting
        # PID for omega
        self.iTerm_w += d_theta
        self.dTerm_w = d_theta - self.previous_dTheta
        angular_velocity = (
                self.kp_w * d_theta         +
                self.ki_w * self.iTerm_w    +
                self.kd_w * self.dTerm_w
            )
        self.previous_dTheta = d_theta
        angular_velocity = max(min(angular_velocity, self.max_ang_vel), -self.max_ang_vel)          # Apply a speed limit to prevent overshooting

        if distance_to_goal <= self.distanceTolerance:
            self.get_logger().info('The robot has reached to the waypoint x: %f, y: %f' % (current_x, current_y))
            #move to next waypoint
            self.index += 1
            if current_x <= 0.5 and current_y <=0.5:
                # reached to the poimt
                self.get_logger().info('The robot has reached to end')
                cmd_vel_msg.twist.linear.x = 0
                cmd_vel_msg.twist.angular.z = 0
                self.cmd_vel_pub_.publish(cmd_vel_msg)
            else:
                cmd_vel_msg.twist.linear.x = linear_velocity
                cmd_vel_msg.twist.angular.z = angular_velocity
        else:
            cmd_vel_msg.twist.linear.x = linear_velocity
            cmd_vel_msg.twist.angular.z = angular_velocity
        
        self.cmd_vel_pub_.publish(cmd_vel_msg)
        

def main(args=None):
    rclpy.init(args=args)
    waypoint_tracking  = WaypointTracking()
    rclpy.spin(waypoint_tracking)
    waypoint_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()