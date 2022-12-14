#!/usr/bin/env python
"""
Module:
    FSM
Author:
    Alice Nardelli alice.nardelli98@gmail.com
ROS node to implement the finite state machine. Inside this node is contained the entire simulation
Service Client:
	/random_room_service to get a random room
	/armor_interface_srv to interface with the ontology
	/check_correct_service to check if an hypotesis is correct
	/perceive_hints_service to perceive hints
	/announce_service to announce an hypotesis
Action Client:
	/move_action_server to simulate the movement
"""
import roslib
import rospy
import smach
import smach_ros
import time
import random
from armor_msgs.srv import ArmorDirective
from exprob_ass1.srv import RandomRoom , CheckCorrect, Announcement, PerceiveHints, AnnouncementRequest
from exprob_ass1.msg import MoveAction, MoveGoal
import actionlib
import actionlib_msgs




    
   


def ontology_interaction(
    command,
    primary_command_spec,
    secondary_command_spec,
     arg):
     '''
            Description of the ontology_interaction function:
            This function is used to interface with the ontology through the aRMOR action server. It get as argument all fields of the ArmorDirective message to fill.
            Args:
               command
               primary_command_spec
               secondary_command_spec
               arg
            Returns:
               msg(ArmorDirectiveRes)
       
     ''' 
     global client_armor
     rospy.wait_for_service('armor_interface_srv')
     
     msg = ArmorDirective()
     msg.client_name = 'tutorial'
     msg.reference_name = 'ontoTest'
     msg.command = command
     msg.primary_command_spec = primary_command_spec
     msg.secondary_command_spec = secondary_command_spec
     msg.args = arg
     resp = client_armor(msg)
     
     return resp


def menage_response(st):
            '''
                     Description of the menage_response function:
                     This function is used to menage the strings retrieved by aRMOR service
                     Args:
                       st: string to manage
                     Returns:
                       st: arranged string
       
            ''' 
            st = st.replace("<http://www.emarolab.it/cluedo-ontology#", "")
            st = st.replace(">", "")
     
            return st


class Inside_Room(smach.State):
    """
            A class used to represent the behavior of the robot when is inside a room
            ...

            Methods
            -------
            init(outcomes=['exit_from_room'],
                 input_keys=['room_in'],
                 output_keys=['room_out'])
                initialize the state
            execute(userdata)
               implement the effective behaviour
    """
    def __init__(self):
        
    	smach.State.__init__(self,
                             outcomes=['exit_from_room'],
                             input_keys=['room_in'],
                             output_keys=['room_out'])

    def execute(self, userdata):
        '''
                 Description of the execute method:
                 When the  robot enter in the room it can perceive or not an hint.
                 If an hint is perceived it reason about that.
                 if it belongs to an hypotesis already check as incorrect it discard it.
                 Otherwise it add the perceived hint to theontology
                 Anyway it exit from room
                 
                     Args:
                       userdata
                       to store the variables between states
                     
                     Returns:
                       'exit_from_room': str
                       string used to pass to the next state
       
        ''' 
        global client_move, client_rnd_room, client_check_correct, client_announce, client_perceive_hints
        time.sleep(1)
        #get the actual room
        actual_room = rospy.get_param(userdata.room_in)
        #state that it entered in that specific room
        rospy.loginfo('I am inside ' + actual_room[0])
        rospy.wait_for_service('perceive_hints_service')
        #perceive one hint
        perceived_hint = client_perceive_hints()
        #if an hint has been perceived the field perceived of the PerceiveHints message is 1
        if perceived_hint.perceived == 1:
            #ask the incorrect hypotesis collected
            r=ontology_interaction('QUERY','IND','CLASS',['INCORRECT'])
            
            incorrect=[]
            for i in range(len(r.armor_response.queried_objects)):
                     st = menage_response(r.armor_response.queried_objects[i])
                     incorrect.append(st)

            #if the perceived object belong to an hypotesis already checked as incorrect robot will discard it
            if perceived_hint.hypotesis in incorrect:
                 rospy.loginfo('The perceived hint has an id associated to an incorrect hypotesis: I will discard it')
            #otherwise it add the object to the ontology
            else:
             r1 = ontology_interaction('ADD', 'OBJECTPROP', 'IND', [perceived_hint.description, perceived_hint.hypotesis, perceived_hint.value])
            
             if perceived_hint.description=='where':
               resp=ontology_interaction('ADD','IND','CLASS',[perceived_hint.value,'PLACE'])
             elif perceived_hint.description=='what':
               resp=ontology_interaction('ADD','IND','CLASS',[perceived_hint.value,'WEAPON'])
             else:
               resp=ontology_interaction('ADD','IND','CLASS',[perceived_hint.value,'PERSON'])
             #disjoint all element of one classes is necessarily for having multiple hints of a class belonging to the same hypotesis
             resp2=ontology_interaction('DISJOINT','IND','CLASS',['PERSON'])
             resp2=ontology_interaction('DISJOINT','IND','CLASS',['PLACE'])
             resp2=ontology_interaction('DISJOINT','IND','CLASS',['WEAPON'])
             #reason about the class
             r2 = ontology_interaction('REASON', '', '', [])
        #move to the corridor
        client_move.wait_for_server()
        goal_msg = MoveGoal()
        goal_msg.destination = 'Corridor'
        goal_msg.actual_x = int(actual_room[1])
        goal_msg.actual_y = int(actual_room[2])
        goal_msg.goal_x = 0
        goal_msg.goal_x = 0
        client_move.send_goal(goal_msg)
        client_move.wait_for_result()
        #state that the next room will be the corridor
        userdata.room_out = 'room0'
        rospy.loginfo('I am exit from the ' + actual_room[0])

        return 'exit_from_room'


class Oracle_Room(smach.State):
    """
            A class used to represent the behavior of the robot when is in the oracle room.

            ...

            Methods
            -------
            init(outcomes=['correct','not_correct'],
                 input_keys=['room_in'],
                 output_keys=['room_out'])
                initialize the state
            execute(userdata)
               implement the effective behaviour
    """
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['correct', 'not_correct'],
                             input_keys=['room_in'],
                             output_keys=['room_out'])

    def execute(self, userdata):
        '''
                 Description of the execute method:
                 When the  robot enter in the Oracle room it announce the current hypotesis
                 It ask if the hypotesis is correct
                 if it is it exit the game is concluded
                 Otherwise it moves to the corridor looking for new hints
                 
                     Args:
                       userdata
                       to store the variables between states
                     
                     Returns:
                       'correct': str
                       string used to pass to the next state, in that case the game ended
                       'not correct':str
                       string used to pass to the next state, in that case robot returns looking for hints
       
        ''' 
        global client_move, client_rnd_room, client_check_correct, client_announce, client_perceive_hints
        global incorrect
        
        time.sleep(1)
        #state that robot is inside the Oracle room
        rospy.loginfo('I am inside Oracle Room')
        #it announces the actual hypotesis
        rospy.wait_for_service('announce_service')
        current_hypotesis = rospy.get_param('current_hypotesis')
        msg = AnnouncementRequest()
        msg.who = current_hypotesis[1]
        msg.where = current_hypotesis[2]
        msg.what = current_hypotesis[3]
        a = client_announce(msg)
        #check if the hypotesis is correct
        rospy.wait_for_service('check_correct_service')
        resp = client_check_correct(current_hypotesis[0])
        #if is correct exit from the game
        if resp.correct==1:
             resp=ontology_interaction('SAVE','INFERENCE','',['/root/ros_ws/src/exprob_ass1/cluedo_ontology_inference.owl'])
             return 'correct'
        #otherwise it moves to the corridor
        client_move.wait_for_server()
        goal_msg = MoveGoal()
        goal_msg.destination = 'Corridor'
        goal_msg.actual_x = 10
        goal_msg.actual_y = 10
        goal_msg.goal_x = 0
        goal_msg.goal_x = 0
        client_move.send_goal(goal_msg)
        client_move.wait_for_result()
        userdata.room_out = 'room0'
        #remove from ontology the hypotesis and insert it as incorrect 
        #in such a way the hypotesis will not be considered complete anymore

        r1=ontology_interaction('REMOVE','IND','',[current_hypotesis[0]])
        r3=ontology_interaction('ADD','IND','CLASS',[current_hypotesis[0],'INCORRECT'])
        r2=ontology_interaction('REASON','','',[])
        rospy.loginfo('I am exit from Oracle Room')
        
        return 'not_correct'


# define state Locked
class Out_Room(smach.State):
    """
            A class used to represent the behavior of the robot when is out of a room

            ...

            Methods
            -------
            init(outcomes=['go_to_oracle','move_to_a_room'],
                 input_keys=['room_in'],
                 output_keys=['room_out'])
                initialize the state
            execute(userdata)
               implement the effective behaviour
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['go_to_oracle', 'move_to_a_room'],
                             input_keys=['room_in'],
                             output_keys=['room_out']

                             )

        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        '''
                 Description of the execute method:
                 When the  robot is not in aroom it check if there are new consistent hypotesis to check.
                 If there is not it moves to a random room looking for hints
                 Otherwise it ask to the ontology the hypotesis and insert it in the parameter server
                 Then it moves to oracle room
                 
                     Args:
                       userdata
                       to store the variables between states
                     
                     Returns:
                       'move_to_a_room': str
                       string used to pass to the next state, in that case robot will visit a random room
                       'go_to_oracle':str
                       string used to pass to the next state, in that case robot will go to the oracle
       
        ''' 
        global client_move, client_rnd_room, client_check_correct, client_announce, client_perceive_hints
        while not rospy.is_shutdown():
            time.sleep(1)
            # Disjoit all items belonging to each class
            resp2=ontology_interaction('DISJOINT','IND','CLASS',['PERSON'])
            resp2=ontology_interaction('DISJOINT','IND','CLASS',['PLACE'])
            resp2=ontology_interaction('DISJOINT','IND','CLASS',['WEAPON'])
            #ask for complete hypotesis
            resp_c = ontology_interaction('QUERY', 'IND', 'CLASS', ['COMPLETED'])

            # ask for incostintent hypotesis
            resp_i = ontology_interaction('QUERY', 'IND', 'CLASS', ['INCONSISTENT'])


            # if the length is equal means that there is not consistent
            # hypotesis to check
            if len(
    resp_i.armor_response.queried_objects) == len(
        resp_c.armor_response.queried_objects):
		# extract randomly a room
             
               rospy.wait_for_service('random_room_service')
               random_room_resp = client_rnd_room()
               random_room = random_room_resp.random_room
               #getting room name and position
               room = rospy.get_param(random_room)
               #move to that room
               client_move.wait_for_server()
               goal_msg =MoveGoal()
               goal_msg.destination = room[0]
               goal_msg.actual_x = 0
               goal_msg.actual_y = 0
               goal_msg.goal_x = int(room[1])
               goal_msg.goal_x = int(room[2])
               client_move.send_goal(goal_msg)
               client_move.wait_for_result()
               userdata.room_out = random_room
               return 'move_to_a_room'
            else:
                #otherwise it look for the consistent hypotesis
                complete = []
                
                
                for i in range(len(resp_c.armor_response.queried_objects)):
                     
                     st = menage_response(resp_c.armor_response.queried_objects[i])
                     complete.append(st)
               
                     
                if len(resp_i.armor_response.queried_objects)>0:
                  for i in range(len(resp_i.armor_response.queried_objects)):
                       st = menage_response(resp_i.armor_response.queried_objects[i])
                       complete.remove(st)
                       
			   
		# query about the hypotesis 
                
                consistent_who=ontology_interaction('QUERY','OBJECTPROP','IND',['who',complete[0]])
                
                who=menage_response(consistent_who.armor_response.queried_objects[0])
                consistent_where=ontology_interaction('QUERY','OBJECTPROP','IND',['where',complete[0]])
                where=menage_response(consistent_where.armor_response.queried_objects[0])
                consistent_what=ontology_interaction('QUERY','OBJECTPROP','IND',['what',complete[0]])
                what=menage_response(consistent_what.armor_response.queried_objects[0])
			   
                # store the consistent hypotesis in the parameter server
                rospy.set_param('current_hypotesis',[complete[0],who,where,what])
		# getting  the oracle room posistion 
                room=rospy.get_param('room10')
                #move to the oracle
                client_move.wait_for_server()
                goal_msg = MoveGoal()
                goal_msg.destination=room[0]
                goal_msg.actual_x=0
                goal_msg.actual_y=0
                goal_msg.goal_x=int(room[1])
                goal_msg.goal_x=int(room[2])
                client_move.send_goal(goal_msg)
                client_move.wait_for_result()
                userdata.room_out='room10'
                return 'go_to_oracle'
					 
               
                
		    
            self.rate.sleep
            

        
def main():
    """
      documentation of the main function
      it is used to inizialize all global variables
      then is load the ontology, add the new class incorrect and set all classes as mutually disjoint
      finally the states of the FSM are defined and transition between states too
    
    """ 
    global client_move, client_rnd_room, client_check_correct, client_announce, client_perceive_hints, client_armor
    #init node
    rospy.init_node('FSM')
    #init service client and action client
    client_armor = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)    
    client_move= actionlib.SimpleActionClient('move_action_server', MoveAction)
    client_rnd_room=rospy.ServiceProxy('random_room_service', RandomRoom)
    client_check_correct=rospy.ServiceProxy('check_correct_service', CheckCorrect)
    client_announce=rospy.ServiceProxy('announce_service', Announcement)
    client_perceive_hints=rospy.ServiceProxy('perceive_hints_service', PerceiveHints)
    # load ontology
    ontology=rospy.get_param('ontology')
    ontology_path=rospy.get_param('ontology_path')

    r1=ontology_interaction('LOAD','FILE','',[ontology_path, ontology, 'true', 'PELLET', 'true'])
    #add class incorrect

    r2=ontology_interaction('ADD','CLASS','CLASS',['INCORRECT','HIPOTESIS'])  
    #set all classes as mutually disjoint 
    r3=ontology_interaction('DISJOINT','CLASS','',['PERSON','PLACE'])
    r4=ontology_interaction('DISJOINT','CLASS','',['PLACE','WEAPON'])
    r5=ontology_interaction('DISJOINT','CLASS','',['WEAPON','PERSON'])
    r6=ontology_interaction('DISJOINT','CLASS','',['INCORRECT','PERSON'])
    r7=ontology_interaction('DISJOINT','CLASS','',['WEAPON','INCORRECT'])
    r8=ontology_interaction('DISJOINT','CLASS','',['INCORRECT','PLACE'])
   

    # declare state machine
    sm = smach.StateMachine(outcomes=['game_finished'])
    sm.userdata.actual_room = 'room0'
    # Open the container
    with sm:
        # Add states to the container
        #defined the states and the transistions
        smach.StateMachine.add('OUT_ROOM', Out_Room(), 
                               transitions={'go_to_oracle':'ORACLE_ROOM',
                                            'move_to_a_room':'INSIDE_ROOM'
                                           },
                               remapping={'room_in':'actual_room', 
                                          'room_out':'actual_room'})
                                          
        smach.StateMachine.add('INSIDE_ROOM', Inside_Room(), 
                               transitions={'exit_from_room':'OUT_ROOM'},
                               remapping={'room_in':'actual_room', 
                                          'room_out':'actual_room'}
                               )
                              
        smach.StateMachine.add('ORACLE_ROOM', Oracle_Room(), 
                               transitions={'not_correct':'OUT_ROOM', 
                                            'correct':'game_finished'},
                               remapping={'room_in':'actual_room', 
                                          'room_out':'actual_room'}
                               )


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
