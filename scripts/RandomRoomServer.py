#! /usr/bin/env python
"""
Module: 
   RandomRoomServer

Author:
   Alice Nardelli alice.nardelli98@gmail.com
ROS nodes used to return a random room belonging to the apartment
Service :
   /random_room_service to get the parameter name of the random room
"""
import rospy
import random
from exprob_ass1.srv import RandomRoom,RandomRoomResponse

def random_room_clbk(req):
    '''
      Description of the callback:
      This function retrieves the empty request of the RandomRoom message.
      Args:
         srv(RandomRoom): empty request retrieved by */random_room_service* topic
      Returns:
         srv(RandomRoom): the parameter name of a random chosen room from the ones of the apartment
    '''
    msg=RandomRoomResponse()
    no_room=rospy.get_param("no_room")
    msg.random_room='room'+str(random.randint(1,no_room))
    
    
    return msg


def main():
    #init node
    rospy.init_node('random_room_service')
    #init service
    srv = rospy.Service('random_room_service', RandomRoom, random_room_clbk)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      
        rate.sleep()


if __name__ == '__main__':
    main()
