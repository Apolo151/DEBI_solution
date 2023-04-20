#!/usr/bin/env python3

import move_to_ball
import get_circles
import arm_control
import rospy

balls_coors = move_to_ball.balls_coors

if __name__=="__main__":
    try:
        get_circles.get_circles()
        for ball in balls_coors:
            move_to_ball.go_to_ball(ball[0], ball[1]-0.3)
        rospy.spin()    
    except Exception as e:
        print(e)