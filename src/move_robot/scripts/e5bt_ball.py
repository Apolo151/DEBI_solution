#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math

# define balls coordinates
balls_coors = []
balls_coors.append([0.273150+0.258880, -0.958760]) # green ball
balls_coors.append([0.968200+0.258880, 0.146911]) # blue ball
balls_coors.append([0.861900+0.258880, 1.110773]) # red ball


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
global linear_velocity, angular_velocity, scan_range, min_scan_distance, max_X

# the x-coordinate the robot should not pass (safe margin)
max_X = 1.300000


def go_to_goal(goal_x, goal_y, isball=False):
    '''A GoToGoal approach to move the robot to its goal
    goal_x: ball x coordinate
    goal_y: ball y coordinate'''

    ## Initialize variables ##
    linear_velocity = 0.75 # max linear velocity
    angular_velocity = 0.6 # max angular velocity
    scan_range = 0.5
    min_scan_distance = float('inf')
    vel_msg = Twist()
    
    ## Define needed functions ##
    def distance(goal_x=goal_x, goal_y=goal_y):
        return ((goal_x - robot.x)**2 + (goal_y - robot.y)**2)**0.5

    def linear(linear_velocity):
        linear_velocity = math.tanh(distance()) * linear_velocity
        return linear_velocity

    def angle(goal_x=goal_x, goal_y=goal_y):
        return (math.atan2(goal_y - robot.y, goal_x - robot.x) - robot.theta)

    def angular(angular_velocity):
        angular_velocity = math.tanh(angle()) * angular_velocity
        return angular_velocity
    
    def orient_to_goal(goal_x=goal_x, goal_y=goal_y):
        while abs(angle(goal_x, goal_y)) >= 0.01*math.pi:
            vel_msg.linear.x = 0
            vel_msg.angular.z = angular(angular_velocity)
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
            print("Rotating with velocity: {}".format(vel_msg.angular.z))
    
    def stop_robot():
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_pub.publish(vel_msg)
        rospy.sleep(0.1)
    
    def back_up():
        while distance(0, robot.y) >= 0.10:
            vel_msg.linear.x = -linear_velocity*0.5
            vel_msg.angular.z = 0
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
        stop_robot()
        rospy.sleep(0.1)
        
    
    '''move the robot'''
    # alter robot orientation to face the ball
    orient_to_goal()

    # give the robot a small velocity to start moving (prevents orientation error)
    vel_msg.linear.x = 0.03
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(1)

    # if I am pushing a ball, move with higher speed till I reach the line
    if isball:
        while distance(max_X, robot.y) >= 0.10:
            vel_msg.linear.x = linear_velocity*0.85
            if robot.x < goal_x:
                vel_msg.angular.z = angular(angular_velocity)
            else:
                vel_msg.angular.z = 0
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
            print("Moving with velocity: {}".format(vel_msg.linear.x))

        # Back up a bit after pushing the ball
        stop_robot()
        back_up()
        
    # else I am adjusting for a ball, move with stidy speed
    else:
        while distance(robot.x, goal_y) >= 0.10:
            vel_msg.linear.x = linear(linear_velocity)  
            vel_msg.angular.z = 0
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
            print("Moving with velocity: {}".format(vel_msg.linear.x))
        rospy.sleep(0.1)
 
    stop_robot()
    print("Reached the goal")

if __name__ == '__main__':
    try:
        for ball in balls_coors:
            # adjust in y-axis
            go_to_goal(0, ball[1])
            # go to ball
            go_to_goal(ball[0], ball[1], True)
            rospy.sleep(0.2)
        go_to_goal(0, 0) # go back to the starting point
        rospy.spin()     
    except rospy.ROSInterruptException:
        pass
