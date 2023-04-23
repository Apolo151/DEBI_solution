#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math
#import get_circles

global ROBOT_STATE
## STATES = ["searching", "fixated", "returning"]
ROBOT_STATE = "searching"

# define ball coordinates
balls_coors = []

balls_coors.append([0, 0]) # origin
#balls_coors.append([0.861900+0.258880, -0.319573]) # red ball
#balls_coors.append([0.973150+0.258880, 0.196194]) # green ball
#balls_coors.append([0.968200+0.258880, -0.758848]) # blue ball

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

def stop_robot():
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_pub.publish(vel_msg)
        rospy.sleep(0.1)
    
def back_up():
    global vel_msg
    vel_msg.linear.x = -0.25*linear_velocity
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(1.2)
    stop_robot()

def search_for_balls():
    vel_msg.linear.x = 0
    # rotate right 
    vel_msg.angular.z = angular_velocity*0.3*-1
    print(vel_msg.angular.z)
    # Publish the velocity message
    velocity_pub.publish(vel_msg)

def fixate_ball_in_frame(msg):
    #global ROBOT_STATE
    # middle of image is 240 in x
    if(abs(msg.point.x-240) > 20):
        vel_msg.linear.x = 0
        if msg.point.x > 240: 
            vel_msg.angular.z = math.tanh(abs(msg.point.x-240)) * angular_velocity*-1*0.1
        else: 
            vel_msg.angular.z = math.tanh(abs(msg.point.x-240)) * angular_velocity*0.1
        ##
        velocity_pub.publish(vel_msg)
        rospy.sleep(0.01)
    else:
        vel_msg.linear.x = 0.2*linear_velocity
        vel_msg.angular.z = 0
        # Publish the velocity message
        velocity_pub.publish(vel_msg)


def circles_callback(msg):
    global ROBOT_STATE
    if ROBOT_STATE=="searching":
        if msg.point.x != -1:
            ROBOT_STATE = "fixated"
            stop_robot()
            print("fixated")
        else:
            search_for_balls()
    elif ROBOT_STATE=="fixated":
        if msg.point.x != -1:
            print("coors: ", msg.point.x, msg.point.y)
            fixate_ball_in_frame(msg)
        #else:
            #ROBOT_STATE = "searching"

    



# Initialize ROS node
rospy.init_node('move_turtlebot3', anonymous=True)

# Initialize publishers and subscribers
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
circles_sub = rospy.Subscriber('/circles_coors', PointStamped, circles_callback)

# Initialize variables
robot = DifferentialRobot(0.033, 0.287)  # create DifferentialRobot object with wheel radius and distance
global linear_velocity, angular_velocity, scan_range, min_scan_distance, max_X, line_x, wall_y

# the x coordinate the robot should not pass (safe margin)
MAX_X = 1.410000
LINE_X = 1.570000
WALL_Y = 1.410000
# Initialize variables
linear_velocity = 0.75 # max linear velocity
angular_velocity = 0.65 # max angular velocity
scan_range = 0.5
min_scan_distance = float('inf')
vel_msg = Twist()

def go_to_ball(goal_x, goal_y, pushing_ball = False):
    '''A GoToGoal approach to move the robot to the ball
    goal_x: ball x coordinate
    goal_y: ball y coordinate'''

    # Define needed functions
    def distance():
        return ((goal_x - robot.x)**2 + (goal_y - robot.y)**2)**0.5

    def linear(linear_velocity):
        linear_velocity = math.tanh(distance()) * linear_velocity
        return linear_velocity

    def angle(goal_x = goal_x, goal_y = goal_y):
        return (math.atan2(goal_y - robot.y, goal_x - robot.x) - robot.theta)

    def angular(angular_velocity):
        angular_velocity = math.tanh(angle()) * angular_velocity
        #angular_velocity = angle() * angular_velocity
        return angular_velocity
    
    
    
    '''move the robot'''
    # alter robot orientation to face the ball


    while abs(angle()) >= 0.02*math.pi:
        vel_msg.linear.x = 0
        vel_msg.angular.z = angular(angular_velocity)
        print(vel_msg.angular.z)
        # Publish the velocity message
        velocity_pub.publish(vel_msg)

    vel_msg.linear.x = 0.03
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(1)
    
    # move towards the ball
    while distance() >= 0.14:
        vel_msg.linear.x = linear(linear_velocity)
        vel_msg.angular.z = angular(angular_velocity)
        # Publish the velocity message
        velocity_pub.publish(vel_msg)
        print("MOVING NOW")
    stop_robot()

    # Sleep for 0.1 seconds
    rospy.sleep(0.1)
    if goal_x != 0 and pushing_ball == False:
        # push the ball to the wall
        go_to_ball(goal_x, WALL_Y, True)

        # alter robot orientation to face the line
        while abs(angle(LINE_X, robot.y)) >= 0.02*math.pi:
            vel_msg.linear.x = 0
            vel_msg.angular.z = angular(angular_velocity)
            print(vel_msg.angular.z)
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
        stop_robot()

        ## push ball towards the line 
        while abs(robot.x - MAX_X) >= 0.09:
            vel_msg.linear.x = linear_velocity*0.8
            vel_msg.angular.z = 0
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
            print("MOVING NOW")
        stop_robot()
        # Back up a bit
        back_up()

if __name__=="__main__":
    try:
        rospy.spin()     
    except Exception as e:
        print(e)