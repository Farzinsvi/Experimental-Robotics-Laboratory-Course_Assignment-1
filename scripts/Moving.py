#!/usr/bin/env python
'''
Module:
   Moving
Author:
  Alice Nardelli alice.nardelli98@gmail.com
ROS nodes used to similate robot movement
Action Service :
  /move_action_server to simulate the robot movement from a starting point to a goal one
'''
import sys
import rospy
import actionlib
import exprob_ass1.msg
import math
import time

_as = None


def action_clbk(req):
    '''
    Description of the callback:
    This function retrieves the goal of the custom MoveAction. Inside the goal are specified the actual and the goal position of the robot and the name of the destination room.
    Movement is simulate as a wait procedure. The wait is proportionale to the length of the path. The path is a straigth line between the two points. 
    The action continously provides as feedbact actual (x,y,yaw) of the robot.
    Args:
       msg(MoveGoal): goal retrieved by */move_action_server/goal* topic
    Returns:
       msg(MoveFeedback): actual (x,y,yaw) of the robot published on */move_action_server/feedback* topic 
       msg(MoveResult): true
    ''' 
    #compute angle of the trajectory
    theta=math.atan(abs(req.actual_y-req.goal_y)/abs(req.actual_x-req.goal_x)) 
    #state where robot is going
    rospy.loginfo('I am going to the '+ req.destination)
    #compute the distance
    distance=math.sqrt(pow(req.actual_y-req.goal_y, 2)+pow(req.actual_x-req.goal_x, 2))
    counter=distance 
    #start publishing feedback
    _fb = exprob_ass1.msg.MoveFeedback()
    _fb.feed_x= req.actual_x
    _fb.feed_y= req.actual_y
    _fb.yaw=theta
    _as.publish_feedback(_fb)
           
    while distance>0:
		   distance=distance-1
                   #for each meter it sleeps 1s
		   rospy.sleep(1)
                   #update feedback
		   if distance>0:
			   if req.actual_y-req.goal_y>0:
				   _fb.feed_y=_fb.feed_y-math.sin(theta)
			   else:
				   _fb.feed_y=_fb.feed_y+math.sin(theta)
			   if req.actual_x-req.goal_x>0:
				   _fb.feed_x=_fb.feed_x-math.cos(theta)
			   else:
				   _fb.feed_x=_fb.feed_x+math.cos(theta)
		 
		   else:
			   _fb.feed_x=req.goal_x
			   _fb.feed_y=req.goal_y
			   
		   _fb.yaw=theta   
		   _as.publish_feedback(_fb)
    #when it arrives it announce it	   
    rospy.loginfo('I am arrived to the '+req.destination)
    _as.set_succeeded() 

if __name__ == '__main__':

       rospy.init_node('talker', anonymous=True)
       _as = actionlib.SimpleActionServer('move_action_server', exprob_ass1.msg.MoveAction,execute_cb=action_clbk, auto_start=False) 
       _as.start()  
       rospy.spin()
