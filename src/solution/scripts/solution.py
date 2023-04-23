#!/usr/bin/env python3

import move_to_ball
import get_circles
#import arm_control
import rospy

balls_coors = move_to_ball.balls_coors




'''Setup Subs and Pubs'''


if __name__=="__main__":
    try:
        curr_circles = None
        search_for_balls(curr_circles)
        for ball in balls_coors:
            move_to_ball.go_to_ball(ball[0], ball[1]-0.3)
        rospy.spin()    
    except Exception as e:
        print(e)