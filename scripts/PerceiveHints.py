#! /usr/bin/env python
"""
Module:
  PerceiveHints
Author:
  Alice Nardelli alice.nardelli98@gmail.com
ROS nodes used for simulate the behavior of the robot when it looks for hint
Service :
  /perceive_hints_service to get the ID of the hypotesis to check
"""
import rospy
import random
from exprob_ass1.srv import PerceiveHints, PerceiveHintsResponse


def perc_clbk(req):
    '''
           Description of the callback:
           This function retrieves the empty request of the PerceiveHints message. Then it perceive if any an hint. and return it
           Args:
              srv(PerceiveHints): empty request retrieved by */perceive_hints_service* topic
           Returns:
              srv(PerceiveHints): the hints perceived if any
    '''
    msg = PerceiveHintsResponse()
    #random extract if perceive or not an hint
    perceived = random.randint(0, 3)
    #if not perceive anything say that and return
    if perceived == 0:
        msg.perceived = 0
        rospy.loginfo('Nothing perceived')
    #otherwise extract randomly an hint from parameter server
    else:
        #get the number of hint
        no_hints = rospy.get_param("/no_hints")
        #extract randomly one hint from parameter server
        rndm_hint = random.randint(1, no_hints)
        perceived_hint = rospy.get_param("/hint" + str(rndm_hint))
        #return the perceived hint and affirm it   
        msg.perceived = 1
        msg.hypotesis = perceived_hint[0]
        msg.description = perceived_hint[1]
        msg.value = perceived_hint[2]
        rospy.loginfo('Perceived '+ msg.hypotesis+':'+msg.description+' '+msg.value )
          

    return msg


def main():
    # init node
    rospy.init_node('perceive_hints_service')
    # init service
    srv = rospy.Service('perceive_hints_service', PerceiveHints, perc_clbk)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
