#! /usr/bin/env python
'''
Module:
  CheckCorrect
Author:
   Alice Nardelli alice.nardelli98@gmail.com
ROS nodes used for check if the current consistent hypothesis is correct or not.
Service :
   /check_correct_service to get the ID of the hypotesis to check
'''
import rospy
import random
from exprob_ass1.srv import CheckCorrect


def check_correct_clbk(req):
    '''
           Description of the callback:
           This function retrieves the request field of the CheckCorrect message. Inside the custom message is present the ID of the current hypotesis.
           This ID is compared with the ID of the correct ID stored in the parameter server.
           Args:
              srv(CheckCorrect): data retrieved by */check_correct_service* topic
           Returns:
              srv(CheckCorrect): 1 if the hypotesis is correct 0 otherwise
    '''
    hypotesis = req.hypotesis
    if hypotesis == rospy.get_param('/correct_hypotesis'):

        rospy.loginfo(hypotesis + ' hypotesis is correct!! ')
        return 1

    rospy.loginfo(hypotesis + ' hypotesis is not correct ')
    return 0


def main():
    # init node
    rospy.init_node('check_correct_service')
    # init service
    srv = rospy.Service(
        'check_correct_service',
        CheckCorrect,
        check_correct_clbk)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
