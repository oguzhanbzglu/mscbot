#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math as m

#TODO: Get the waypoint from a different folder as txt 

class  WaypointTracking(Node):
    def __init__(self):
        super().__init__('waypoint_tracking')

        # publishers and subscribers
        self.odom_sub_ = self.create_subscription(Odometry, 'kinematics_controller/odom', self.odomCallback, 10)
        self.cmd_vel_pub_ = self.create_publisher(TwistStamped, 'kinematics_controller/cmd_vel', 10)

        self.get_logger().info('The Waypoint Tracking node has been started!')

        #location of the file
        # self.path_directory = '/home/usr/mscbot_ws/src/mscbot_utils/path/waypoint.txt'
        # self.f = open(self.path_directory, 'r')
        # self.waypoint = self.f.readlines()
        # self.f.close()

        self.distanceTolerance = 0.5
        self.waypoint = [
                        (1.0, 1.0),
                        (5.0, 5.0),
                        (5.0, 7.0),
                        (9.0, 2.0),
                        (9.0, 1.0),
                        (5.0, 5.0)
                            ]

        self.index = 0
        self.x_goal = 0
        self.y_goal = 0
    
    def odomCallback(self, msg):

        cmd_vel_msg = TwistStamped()
        
        #robot's current pose
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_theta = msg.pose.pose.orientation.z

        #goal pose
        self.x_goal = self.waypoint[self.index][0]
        self.y_goal = self.waypoint[self.index][1]

        #position error
        d_x = self.x_goal - current_x
        d_y = self.y_goal - current_y

        theta_goal = m.atan2(d_y, d_x)
        d_theta = theta_goal - current_theta

        #normalizing the heading angle between +pi, -pi
        d_theta = m.atan2(m.sin(d_theta), m.cos(d_theta))

        distance_to_goal = m.sqrt(d_x**2 + d_y**2)
        #print(distance_to_goal)
        

        if distance_to_goal <= self.distanceTolerance:
            self.get_logger().info('The robot has reached to the waypoint x: %f, y: %f' % (current_x, current_y))
            #move to next waypoint
            self.index += 1
            if self.index  > 5:
                # reached to the poimt
                self.get_logger().info('The robot has reached to end')
                cmd_vel_msg.twist.linear.x = 0
                cmd_vel_msg.twist.angular.z = 0
                self.cmd_vel_pub_.publish(cmd_vel_msg)
            else:
                cmd_vel_msg.twist.linear.x = 0.2
                cmd_vel_msg.twist.angular.z = max(min(d_theta, 1.0), -1.0)
        else:
            cmd_vel_msg.twist.linear.x = 0.2
            cmd_vel_msg.twist.angular.z = max(min(d_theta, 1.0), -1.0)
        
        self.cmd_vel_pub_.publish(cmd_vel_msg)
        

def main(args=None):
    rclpy.init(args=args)
    waypoint_tracking  = WaypointTracking()
    rclpy.spin(waypoint_tracking)
    waypoint_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()