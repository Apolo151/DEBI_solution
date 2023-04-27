#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math

def odom_callback(odom_msg):
    # Update current position and orientation of the robot
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    robot.x = position.x
    robot.y = position.y
    robot.theta = yaw

def scan_callback(scan_msg):
    # Find the minimum distance in the scan range
    global min_scan_distance
    min_scan_distance = min(scan_msg.ranges[:len(scan_msg.ranges)//3])

# Initialize ROS node
rospy.init_node('move_turtlebot3', anonymous=True)

# Initialize publishers and subscribers
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)



# Initialize Robot and Variables
robot = DifferentialRobot(0.033, 0.287)  # create DifferentialRobot object with wheel radius and distance
global linear_velocity, angular_velocity, scan_range, min_scan_distance, max_X, vel_msg

# the x-coordinate the robot should not pass (safe margin)
max_X = 1.480000

## Initialize variables ##
linear_velocity = 0.75 # max linear velocity
angular_velocity = 0.65 # max angular velocity
scan_range = 0.5
min_scan_distance = float('inf')
vel_msg = Twist()

def stop_robot():
    global vel_msg
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(0.2)

def rotate():
    vel_msg.linear.x = 0
    vel_msg.angular.z = angular_velocity*0.5
    print("Rotating with velocity: {}".format(vel_msg.angular.z))
    velocity_pub.publish(vel_msg)
    rospy.sleep(0.1)


def go_to_goal(goal_x, goal_y, isball=False):
    '''A GoToGoal approach to move the robot to its goal
    goal_x: ball x coordinate
    goal_y: ball y coordinate'''

    
    
    ## Define needed functions ##
    def distance(goal_x=goal_x, goal_y=goal_y):
        return ((goal_x - robot.x)**2 + (goal_y - robot.y)**2)**0.5

    def linear(linear_velocity, goal_x=goal_x, goal_y=goal_y):
        linear_velocity = math.tanh(distance(goal_x, goal_y)) * linear_velocity
        return linear_velocity

    def angle(goal_x=goal_x, goal_y=goal_y):
        return (math.atan2(goal_y - robot.y, goal_x - robot.x) - robot.theta)

    def angular(angular_velocity, goal_x=goal_x, goal_y=goal_y):
        angular_velocity = math.tanh(angle(goal_x, goal_y)) * angular_velocity
        return angular_velocity
    
    def orient_to_goal(goal_x=goal_x, goal_y=goal_y):
        while abs(angle(goal_x, goal_y)) >= 0.012*math.pi:
            vel_msg.linear.x = 0
            vel_msg.angular.z = angular(angular_velocity, goal_x, goal_y)
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
            print("Rotating with velocity: {}".format(vel_msg.angular.z))

    def after_orient(goal_x=goal_x, goal_y=goal_y):
        # give the robot a small velocity to start moving (prevents orientation error)
        vel_msg.linear.x = 0.05*linear_velocity
        vel_msg.angular.z = angular(angular_velocity, goal_x, goal_y)
        velocity_pub.publish(vel_msg)
        rospy.sleep(0.15)
    
    def back_up():
        while distance(0, robot.y) >= 0.10:
            vel_msg.linear.x = -linear_velocity*0.5
            vel_msg.angular.z = 0
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
        stop_robot()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            rotate()
        stop_robot()
        rospy.spin()
    except Exception as e:
        print(e)
