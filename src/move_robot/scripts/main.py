#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math

class MoveTurtleBot3:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('move_turtlebot3', anonymous=True)

        # Initialize publishers and subscribers
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Initialize variables
        self.robot = DifferentialRobot(0.033, 0.287)  # create DifferentialRobot object with wheel radius and distance
        self.goal_x = 0.973147+0.258880
        self.goal_y = 0.196194
        self.linear_velocity = 0.6
        self.angular_velocity = 0.8
        self.scan_range = 0.5
        self.min_scan_distance = float('inf')
        self.vel_msg = Twist()

    def odom_callback(self, odom_msg):
        # Update current position and orientation of the robot
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot.x = position.x
        self.robot.y = position.y
        self.robot.theta = yaw

    def scan_callback(self, scan_msg):
        # Find the minimum distance in the scan range
        self.min_scan_distance = min(scan_msg.ranges[:len(scan_msg.ranges)//3])

    def distance(self):
        return ((self.goal_x - self.robot.x)**2 + (self.goal_y - self.robot.y)**2)**0.5

    def linear(self):
        linear_velocity = math.tanh(self.distance()) * self.linear_velocity
        return linear_velocity

    def angle(self):
        return (math.atan2(self.goal_y - self.robot.y, self.goal_x - self.robot.x) - self.robot.theta)

    def angular(self):
        angular_velocity = math.tanh(self.angle()) * self.angular_velocity
        return angular_velocity

    def move_to_goal(self):

        # Limit the linear velocity if obstacle is too close
        # if self.min_scan_distance < self.scan_range:
        #     linear_velocity = min(linear_velocity, self.min_scan_distance - self.scan_range)

        # # Set the wheel velocities of the robot using the differential drive model
        # left_wheel_velocity = (2 * linear_velocity - angular_velocity * self.robot.wheel_distance) / (2 * self.robot.wheel_radius)
        # right_wheel_velocity = (2 * linear_velocity + angular_velocity * self.robot.wheel_distance) / (2 * self.robot.wheel_radius)
        # self.robot.set_wheel_velocities(left_wheel_velocity, right_wheel_velocity)
        
        while self.distance() >= 0.01:
            self.vel_msg.linear.x = self.linear()
            self.vel_msg.angular.z = self.angular()
            # Publish the velocity message
            self.velocity_pub.publish(self.vel_msg)
            print(self.vel_msg)

        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_pub.publish(self.vel_msg)

        # Sleep for 0.1 seconds
        rospy.sleep(0.1)

        rospy.spin()

if __name__ == '__main__':
    try:
        robot = MoveTurtleBot3()
        robot.move_to_goal()
    except rospy.ROSInterruptException:
        pass