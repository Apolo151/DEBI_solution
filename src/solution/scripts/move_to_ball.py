#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math

global ROBOT_STATE, linear_velocity, angular_velocity, scan_range, min_scan_distance, MAX_X, LINE_X, WALL_Y
global goal_x, goal_y
goal_x = 2.700000
goal_y = 0.0

ROBOT_STATE = "Searching"
## STATES = ["SearchingForBall", "Fixated", "PickingBall",
#  "GoingToSideWall", "PlayingGolf", "ReturningBack"]

# Initialize variables
MAX_X = 1.410000 # the x coordinate the robot should not pass (safe margin)
LINE_X = 1.570000
WALL_Y = 1.350000
linear_velocity = 0.75 # max linear velocity
angular_velocity = 0.65 # max angular velocity
scan_range = 0.5
min_scan_distance = float('inf')
vel_msg = Twist()

# define ball coordinates
balls_coors = []
balls_coors.append([0, 0]) # origin
#balls_coors.append([0.861900+0.258880, -0.319573]) # red ball
#balls_coors.append([0.973150+0.258880, 0.196194]) # green ball
#balls_coors.append([0.968200+0.258880, -0.758848]) # blue ball

def stop_robot():
    global vel_msg
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(0.2)
    
def back_up():
    global vel_msg
    vel_msg.linear.x = -0.25*linear_velocity
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(1)
    stop_robot()

def search_for_balls():
    global vel_msg
    vel_msg.linear.x = 0
    # rotate right 
    vel_msg.angular.z = angular_velocity*0.3*-1
    print(vel_msg.angular.z)
    # Publish the velocity message
    velocity_pub.publish(vel_msg)

def fixate_ball_in_frame(msg, moving=False):
    global vel_msg
    # middle of image is 240 in x
    vel_msg.linear.x = 0.3*linear_velocity
    vel_msg.angular.z = 0
    if msg.point.x != -1:
        if(abs(msg.point.x-240) > 10 or moving):
            if msg.point.x > 240: 
                vel_msg.angular.z = math.tanh(abs(msg.point.x-240)) * angular_velocity*-1*0.1
            else: 
                vel_msg.angular.z = math.tanh(abs(msg.point.x-240)) * angular_velocity*0.1
        else:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.2*linear_velocity
            #vel_msg.angular.z = 0
            # Publish the velocity message
    velocity_pub.publish(vel_msg)


# Initialize Robot
robot = DifferentialRobot(0.033, 0.287)  # create DifferentialRobot object with wheel radius and distance


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

# Define needed functions
def distance(goal_x=goal_x, goal_y=goal_y):
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

def go_to_goal(goal_x=goal_x, goal_y=goal_y, pushing_ball = False):
    '''A GoToGoal approach to move the robot to the ball
    goal_x: ball x coordinate
    goal_y: ball y coordinate'''

    '''move the robot'''
    global vel_msg
    # alter robot orientation to face the ball
    while abs(angle(goal_x=goal_x, goal_y=goal_y)) >= 0.02*math.pi:
        vel_msg.linear.x = 0
        vel_msg.angular.z = angular(angular_velocity)
        #print(vel_msg.angular.z)
        print(angle())
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
    
    '''## push ball towards the line 
    while abs(robot.x - MAX_X) >= 0.09:
        vel_msg.linear.x = linear_velocity*0.8
        vel_msg.angular.z = 0
        # Publish the velocity message
        velocity_pub.publish(vel_msg)
        #print("MOVING NOW")
    stop_robot()
    # Back up a bit
    back_up()'''

def circles_callback(msg):
    global ROBOT_STATE
    if ROBOT_STATE=="Searching":
        if msg.point.x != -1:
            ROBOT_STATE = "Fixated"
            stop_robot()
        else:
            search_for_balls()
    elif ROBOT_STATE=="Fixated":
        if msg.point.x != -1:
            fixate_ball_in_frame(msg)
            print("coors: ", msg.point.x, msg.point.y)
            if msg.point.z > 70:
                rospy.sleep(1)
                stop_robot()
                ROBOT_STATE = "PickingBall"
        #else:
            #ROBOT_STATE = "searching"
    elif ROBOT_STATE=="PickingBall":
        ## pick_ball() (TODO)
        res = True #validate you picked ball
        if res==True:
            back_up()
            stop_robot()
            ROBOT_STATE = "GoingToSideWall"
    elif ROBOT_STATE=="GoingToSideWall":
        global goal_x, goal_y
        goal_x=robot.x
        if robot.y < 0:
            goal_y=-WALL_Y
        else:
            goal_y=WALL_Y
        go_to_goal()
        ROBOT_STATE = "PlayingGolf"
    elif ROBOT_STATE=="PlayingGolf":
        #play_golf()
        ROBOT_STATE = "ReturningBack"
    else:
        back_up()
        ROBOT_STATE = "Searching"
    
    state_pub.publish(ROBOT_STATE)
    print(ROBOT_STATE)
    print()


# Initialize Node
rospy.init_node('move_turtlebot3', anonymous=True)
# Initialize publishers and subscribers
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
circles_sub = rospy.Subscriber('/circles_coors', PointStamped, circles_callback)
state_pub = rospy.Publisher('/robot_state', String, queue_size=10)

if __name__=="__main__":
    try:
        rospy.spin()     
    except Exception as e:
        print(e)
