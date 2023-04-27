#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math

# Initialize Node
rospy.init_node('move_turtlebot', anonymous=True)

from arm_control import *

global ROBOT_STATE, linear_velocity, angular_velocity, scan_range, min_scan_distance
global MAX_X, LINE_X, WALL_RIGHT_Y, WALL_LEFT_Y, END_X
global goal_x, goal_y, CAM_MID_X, STOP_RADIUS

goal_x = 1.0
goal_y = 0.0
CAM_MID_X = 320
STOP_RADIUS = 65
FLUSH_BALL = False

ROBOT_STATE = "Searching"
## STATES = ["SearchingForBall", "FixatingBall", "PickingBall",
#  "GoingToSideWall", "PushingBall", "ReturningBack"]

# Initialize variables
MAX_X = 1.500000 # the x coordinate the robot should not pass (safe margin)
LINE_X = 1.560000 # the x coordinate of the line
WALL_RIGHT_Y = -1.10000
WALL_LEFT_Y = 1.320000
END_X = 3.000000
linear_velocity = 0.8 # max linear velocity
angular_velocity = 0.65 # max angular velocity
scan_range = 0.5
min_scan_distance = float('inf')
vel_msg = Twist()


'''Robot Arm Control'''
## Define control functions
def open_gripper(speed=0.2,acceleration=0.1, angle=0.018):
    gripper_group.gripper_control([angle,angle],speed,acceleration) #open the gripper

def close_gripper(speed=0.2,acceleration=0.1):
    gripper_group.gripper_control([-0.001,-0.001],speed,acceleration) #close the gripper

# put arm in front of the robot
def go_down_arm(speed=0.1,acceleration=0.1):
    TransformationCalculator.put_frame_static_frame(parent_frame_name="base_footprint",child_frame_name="ball_pos",frame_coordinate=[0.118,0.000,0.025,0.0,1.57,0.0])
    pose=TransformationCalculator.transform(parent_id="base_footprint",child_frame_id="ball_pos")
    arm_group.go_to_pose_goal_cartesian(pose,speed,acceleration)

# put arm at the top of the robot
def go_up_arm(speed=0.1,acceleration=0.1):
    TransformationCalculator.put_frame_static_frame(parent_frame_name="base_footprint",child_frame_name="ball_pos",frame_coordinate=[0.000,0.000,0.3,0.0,0.0,0.0])
    pose=TransformationCalculator.transform(parent_id="base_footprint",child_frame_id="ball_pos")
    arm_group.go_to_pose_goal_cartesian(pose,speed,acceleration)

def play_front_golf():
    # Go Down
    go_down_arm()
    ## drop the ball
    open_gripper(angle=0.025)
    ## back arm a bit
    joint_pos=arm_group.get_joint_state()
    arm_group.go_by_joint_angle([joint_pos[0], joint_pos[1],joint_pos[2],joint_pos[3]+math.radians(6)],1,1,angle_is_degree=False)
    close_gripper()
    ## push ball with arm (TODO)
    joint_pos=arm_group.get_joint_state()
    arm_group.go_by_joint_angle([joint_pos[0], joint_pos[1],joint_pos[2],joint_pos[3]+math.radians(-75)],1,1,angle_is_degree=False)

def go_front_gripper():
    arm_group.go_by_joint_angle([math.radians(0),math.radians(64),math.radians(-33),math.radians(8)],0.1,0.1,angle_is_degree=False)

def go_back_gripper():
    arm_group.go_by_joint_angle([math.radians(0),math.radians(64),math.radians(-33),math.radians(62)],0.1,0.1,angle_is_degree=False)

def go_stuck_to_front_gripper():
    arm_group.go_by_joint_angle([math.radians(0),math.radians(47),math.radians(1),math.radians(45)],0.15,0.15,angle_is_degree=False)

def pick_front_ball():
    # Open Grippger
    open_gripper(angle=0.027)
    # Position arm stuck to robot's front
    go_stuck_to_front_gripper()
    #### position arm, move a small distance to make sure ball is stuck to robot body, then pick it up
    msg = Twist()
    msg.linear.x = 0.1
    velocity_pub.publish(msg)
    rospy.sleep(0.5)
    # if close to line, back up
    do_not_cross_line()
    stop_robot()
    close_gripper()
    go_up_arm()

def is_ball_picked():
    gripper_state=gripper_group.get_joint_state()
    print(gripper_state)
    if gripper_state[0] < 0.001:
        return False
    else:
        return True
    

'''Robot Movement'''

def stop_robot(time=0.2):
    global vel_msg
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    rospy.sleep(time)
    
def back_up(time=1, to_mid=False):
    global vel_msg
    vel_msg.linear.x = -0.4*linear_velocity
    vel_msg.angular.z = 0
    if to_mid:
        vel_msg.angular.z = angular_velocity*0.35*-1
        if robot.y > 0:
            vel_msg.angular.z = vel_msg.angular.z*-1
    velocity_pub.publish(vel_msg)
    rospy.sleep(time)
    stop_robot()

def search_for_balls():
    global vel_msg
    vel_msg.linear.x = 0
    # rotate right 
    vel_msg.angular.z = angular_velocity*0.6*-1
    # Publish the velocity message
    velocity_pub.publish(vel_msg)

def fixate_ball_in_frame(msg):
    global vel_msg, ROBOT_STATE
    # middle of image is 320 in x-axis
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    if msg.point.x != -1:
        if(abs(msg.point.x-CAM_MID_X) > 15):
            vel_msg.linear.x = 0
            vel_msg.angular.z = math.tanh(abs(msg.point.x-CAM_MID_X)) * angular_velocity*0.05
            if msg.point.x > CAM_MID_X: 
                vel_msg.angular.z = vel_msg.angular.z*-1
        elif(abs(msg.point.x-CAM_MID_X) > 3):
            #vel_msg.linear.x = linear_velocity*math.tanh(1/abs(msg.point.z*0.3))
            vel_msg.linear.x = abs(STOP_RADIUS-msg.point.z)/STOP_RADIUS*linear_velocity*0.4
            vel_msg.angular.z = math.tanh(abs(msg.point.x-CAM_MID_X)) * angular_velocity*0.01
            if msg.point.x > CAM_MID_X: 
                vel_msg.angular.z = vel_msg.angular.z*-1      
        else:
            vel_msg.angular.z = 0
            vel_msg.linear.x = abs(STOP_RADIUS-msg.point.z)/STOP_RADIUS*linear_velocity*0.5
    
    else:
        ROBOT_STATE = "SEARCHING"
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

def linear(linear_velocity, goal_x=goal_x, goal_y=goal_y):
    linear_velocity = math.tanh(distance(goal_x, goal_y)) * linear_velocity
    return linear_velocity

def angle(goal_x = goal_x, goal_y = goal_y):
    return (math.atan2(goal_y - robot.y, goal_x - robot.x) - robot.theta)

def angular(angular_velocity, goal_x = goal_x, goal_y = goal_y):
    angular_velocity = math.tanh(angle(goal_x, goal_y)) * angular_velocity
    #angular_velocity = angle() * angular_velocity
    return angular_velocity

def go_to_goal(goal_x=goal_x, goal_y=goal_y, pushing_ball = False):
    '''A GoToGoal approach to move the robot to the ball
    goal_x: ball x coordinate
    goal_y: ball y coordinate'''

    '''move the robot'''
    global vel_msg
    orient_to_goal(goal_x, goal_y)
    after_orient(goal_x, goal_y)
    stop_robot()
    # move towards the goal
    while distance(goal_x=goal_x, goal_y=goal_y) >= 0.1:
        vel_msg.linear.x = linear(linear_velocity, goal_x, goal_y)
        vel_msg.angular.z = angular(angular_velocity, goal_x, goal_y)
        # Publish the velocity message
        velocity_pub.publish(vel_msg)
        print("Moving with velocity: ", vel_msg.linear.x)
    stop_robot()
    rospy.sleep(0.1)

def is_heading_front(theta=robot.theta):
    '''Check if the robot is heading towards the front of the robot'''
    if abs(theta) <= math.pi/3:
        return True
    else:
        return False

def orient_to_goal(goal_x=goal_x, goal_y=goal_y):
    '''Orient the robot to face the line'''
    global vel_msg
    # alter robot orientation to face the line
    while abs(angle(goal_x, goal_y)) >= 0.02*math.pi:
        vel_msg.linear.x = 0
        vel_msg.angular.z = angular(angular_velocity, goal_x, goal_y)
        # Publish the velocity message
        velocity_pub.publish(vel_msg)
    stop_robot()

def after_orient(goal_x=goal_x, goal_y=goal_y):
        # give the robot a small velocity to start moving (reduces orientation error)
        vel_msg.linear.x = 0.05*linear_velocity
        vel_msg.angular.z = angular(angular_velocity, goal_x, goal_y)
        velocity_pub.publish(vel_msg)
        rospy.sleep(0.15)

def push_ball():
    '''Push the ball towards the line'''
    global vel_msg
    # move towards the goal
    while abs(robot.x - MAX_X) >= 0.18:
        vel_msg.linear.x = max(linear(linear_velocity, MAX_X, robot.y)*1.5, linear_velocity*0.7)
        # Publish the velocity message
        velocity_pub.publish(vel_msg)   
    stop_robot()

def do_not_cross_line():
    global ROBOT_STATE
    if robot.x >= MAX_X:
        stop_robot()
        back_up(2.2)
        orient_to_goal(robot.x, WALL_RIGHT_Y)
        ROBOT_STATE = "Searching"

'''Edge Cases (TODO)'''
def handle_stuck_to_wall():
    # Orient till prependicular to wall
    if robot.y >= WALL_LEFT_Y:
        vel_msg.linear.x = 0.1*linear_velocity
        vel_msg.angular.z = 0.1*angular_velocity
        if robot.theta > math.pi/2:
            vel_msg.angular.z = -vel_msg.angular.z
    elif robot.y <= -WALL_RIGHT_Y:
        vel_msg.linear.x = 0.1*linear_velocity
        vel_msg.angular.z = -0.1*angular_velocity
        if robot.theta < -math.pi/2:
            vel_msg.angular.z = -vel_msg.angular.z
    velocity_pub.publish(vel_msg)
    rospy.sleep(0.5)
    stop_robot()
    print("TODO")

def handle_ball_in_corner():
    print("TODO")

def circles_callback(msg):
    global ROBOT_STATE, STOP_RADIUS, LAST_RADIUS, vel_msg
    LAST_RADIUS=msg.point.z
    if ROBOT_STATE=="Searching":
        ## if ball is in the middle of the frame, stop robot and go to next state
        if msg.point.x > 150 and msg.point.x < 430:
            # if looking forward, need to check if ball is in opponent's side (half TODO)
            if msg.point.z > 8.5 or not is_heading_front():
                print(msg.point.x)
                ROBOT_STATE = "FixatingBall"
                stop_robot()
        else:
            search_for_balls()
    elif ROBOT_STATE=="FixatingBall":
        if msg.point.x != -1:
            fixate_ball_in_frame(msg)
            print("coors: ", msg.point.x, msg.point.y)
            # if robot is about to cross the line (ball is in opponent's side) back up and search for other balls
            do_not_cross_line()
            # if radius is greater than STOP_RADIUS, stop robot and go to next state
            if msg.point.z > STOP_RADIUS:
                vel_msg.linear.x = 0.11
                vel_msg.angular.z = 0
                #if msg.point.y
                velocity_pub.publish(vel_msg)
                ## move till stuck to ball
                rospy.sleep(1.75)
                stop_robot()
                ROBOT_STATE = "PickingBall"
        #else:
            #ROBOT_STATE = "searching"
    elif ROBOT_STATE=="PickingBall":
        pick_front_ball()
        ###
        res = is_ball_picked()
        if res:
            back_up(0.5)
            stop_robot()
            ROBOT_STATE = "GoingToSideWall"
        else:
            back_up(1.5)
            ROBOT_STATE = "Searching"
    elif ROBOT_STATE=="GoingToSideWall":
        global goal_x, goal_y
        # because if current is x-pos is low, we might not have margin to back up
        goal_x = min(max(robot.x, 0.7), 1.2)
        if robot.y < 0:
            goal_y= WALL_RIGHT_Y
        else:
            goal_y= WALL_LEFT_Y
        go_to_goal(goal_x, goal_y)
        ROBOT_STATE = "PushingBall"
    elif ROBOT_STATE=="PushingBall":
        # Orient to line
        orient_to_goal(LINE_X, robot.y)
        after_orient(LINE_X, robot.y)
        stop_robot()
        ## Leave the ball
        go_down_arm()
        if robot.y > 0:
            orient_to_goal(LINE_X, WALL_LEFT_Y)
        else:
            orient_to_goal(LINE_X, WALL_RIGHT_Y)
        open_gripper()
        go_up_arm()
        # Back and Push
        back_up(1)
        push_ball()
        ROBOT_STATE = "ReturningBack"
    else:
        #go_to_goal(0,0)
        back_up(6, True)
        # look to side wall
        orient_to_goal(robot.x, WALL_RIGHT_Y)
        stop_robot()
        rospy.sleep(0.5)
        ROBOT_STATE = "Searching"   
    state_pub.publish(ROBOT_STATE)
    print(ROBOT_STATE)
       


# Initialize publishers and subscribers
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
circles_sub = rospy.Subscriber('/circles_coors', PointStamped, circles_callback, queue_size=1)
state_pub = rospy.Publisher('/robot_state', String, queue_size=10)

if __name__=="__main__":
    try:
        rospy.spin()     
    except Exception as e:
        print(e)