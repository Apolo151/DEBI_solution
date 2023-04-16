#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from differential_robot import DifferentialRobot
import math


global pump_distance, push_distance
pump_distance = 0.20
push_distance = 0.02


balls_coors = []


balls_coors.append([0.973147+0.258880+push_distance, 0.196194]) # green ball
balls_coors.append([0, 0]) #  return to origin
balls_coors.append([0.861900+0.258880+push_distance, -0.319573]) # red ball
balls_coors.append([0, 0]) #  return to origin
balls_coors.append([0.968200+0.258880+push_distance, -0.758848]) # blue ball
balls_coors.append([0, 0]) #  return to origin



def odom_callback(odom_msg):
    # Update current position and orientation of the robot
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    robot.x = position.x
    robot.y = position.y
    robot.theta = yaw
    print("current x: {}\n ".format(robot.x))

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

# Initialize variables
robot = DifferentialRobot(0.033, 0.287)  # create DifferentialRobot object with wheel radius and distance
global linear_velocity, angular_velocity, scan_range, min_scan_distance



def go_to_ball(goal_x, goal_y):
    linear_velocity = 0.8
    angular_velocity = 5
    scan_range = 0.5
    min_scan_distance = float('inf')
    vel_msg = Twist()
        
    def distance():
        return ((goal_x - robot.x)**2 + (goal_y - robot.y)**2)**0.5

    def linear(linear_velocity):
        linear_velocity = math.tanh(distance()) * linear_velocity
        return linear_velocity

    def angle():
        return (math.atan2(goal_y - robot.y, goal_x - robot.x) - robot.theta)

    def angular(angular_velocity):
        #angular_velocity = math.tanh(angle()) * angular_velocity
        angular_velocity = angle() * angular_velocity
        return angular_velocity

    def move_to_goal():
        while distance() >= 0.10:
            print("goal_x: {}, goal_y: {}".format(goal_x, goal_y))
            if distance() <= pump_distance:
                 vel_msg.linear.x = linear(linear_velocity)*1.3
                 print("I am within pump distance")
            else:
                vel_msg.linear.x = linear(linear_velocity)
                print("I am still not within pump distance")

            
            vel_msg.angular.z = angular(angular_velocity)
            # Publish the velocity message
            velocity_pub.publish(vel_msg)
            #print(vel_msg)

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_pub.publish(vel_msg)

        # Sleep for 0.1 seconds
        rospy.sleep(0.3)

    move_to_goal()


if __name__ == '__main__':
    try:
        for ball in balls_coors:
            go_to_ball(ball[0], ball[1])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
