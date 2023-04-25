#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math
import csv

with open('radius_to_distance.csv', 'w', newline='') as file:
    ### curent factor
    ### radius * 0.433779928 = IRL_Distance
    writer = csv.writer(file)
    writer.writerow(["Radius", "IRL_Distance"])


    global ROBOT_STATE, linear_velocity, angular_velocity, scan_range, min_scan_distance, MAX_X, LINE_X, WALL_Y
    global goal_x, goal_y
    goal_x = 2.700000
    goal_y = 0.0
    global CAMERA_MID
    CAMERA_MID = 320

    # Initialize variables
    MAX_X = 1.410000 # the x coordinate the robot should not pass (safe margin)
    LINE_X = 1.570000
    WALL_Y = 1.350000
    linear_velocity = 0.75 # max linear velocity
    angular_velocity = 0.65 # max angular velocity
    scan_range = 0.5
    min_scan_distance = float('inf')
    vel_msg = Twist()

    def fixate_ball_in_frame(msg, moving=False):
        global vel_msg
        # middle of image is 240 in x
        vel_msg.linear.x = 0.1*linear_velocity
        vel_msg.angular.z = 0
        if msg.point.x != -1:
            if(abs(msg.point.x-CAMERA_MID) > 5):
                if msg.point.x > CAMERA_MID: 
                    vel_msg.angular.z = math.tanh(abs(msg.point.x-CAMERA_MID)) * angular_velocity*0.02*-1
                else: 
                    vel_msg.angular.z = math.tanh(abs(msg.point.x-CAMERA_MID)) * angular_velocity*0.02
            else:
                vel_msg.angular.z = 0.0
                vel_msg.linear.x = 0.1*linear_velocity
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

    def circles_callback(msg):
        fixate_ball_in_frame(msg, moving=True)
        rospy.loginfo("True_dis: {}, radius: {}".format(distance(), msg.point.z))
        if msg.point.z != -1:
            writer.writerow([msg.point.z, distance()])

    # Initialize Node
    rospy.init_node('radius', anonymous=True)
    # Initialize publishers and subscribers
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    circles_sub = rospy.Subscriber('/circles_coors', PointStamped, circles_callback)

    if __name__=="__main__":
        try:
            rospy.spin()     
        except Exception as e:
            print(e)
