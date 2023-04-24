#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point
import re, random

if __name__ == '__main__':

    rospy.init_node("three_balls")
    
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    
    #Load ball's sdf. Note that the file is no exactly correct- since one tag is missing its content. Said content is added in the next block.
    with open("../world/ball.xml", "r") as f:
        ball_xml = f.read()
    
    #To later randomize color of generated spheres.
    match = re.search("<ambient>", ball_xml)
    
    #Randomize pose and spawn.
    x_min, x_max = -0.50, 1.57
    y_min, y_max = -1.40, 1.53
    
    for i in range(0, 3):
    
        ran_y = random.uniform(y_min, y_max)
        ran_x = random.uniform(x_min, x_max)
        
        model_name   =   "ball_{0}".format(round(ran_x + ran_y, 2))
        print("Spawning model:", model_name)
        
        model_pose = Pose()
        model_pose.position    = Point(ran_x, ran_y, 0.055)
        model_pose.orientation = Quaternion(0, 0, 0, 1)
        
            
        r = str(round(random.random(), 2)) + " " 
        g = str(round(random.random(), 2)) + " " 
        b = str(round(random.random(), 2)) + " "    
        coloured_ball_xml = ball_xml[:match.end(0)] + r + g + b + ball_xml[match.end(0):]
        
        spawn_model(model_name, coloured_ball_xml, "", model_pose, "world")
