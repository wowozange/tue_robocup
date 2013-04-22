#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_emergency')
import rospy
import smach
import math
import cv
import roslib
#import sys
import roslib.packages as p

from robot_skills.amigo import Amigo
import robot_smach_states as states
from robot_smach_states.util.startup import startup

from std_msgs.msg import String
from speech_interpreter.srv import GetContinue
from speech_interpreter.srv import GetYesNo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#import states_new as states 
from psi import *



#########################################
# Created by: Erik Geerts, Teun Derksen #
#########################################

#################
## TODO LIST!! ##
#################

# Implement
#   - Detect smoke
#   - Detect people
#   - Questions to person
#   - Write data/pictures to pdf
#   - Guiding person to exit
#   - ..

##########################################
############## What to run: ##############
############ updated 15-4-2013 ###########
##########################################
# - astart
# - amiddle
# - roslaunch create_speech_files speech.launch   (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - !! Wait for speech.launch to finish before !!
#   !!   launching speech interpreter          !!
#   roslaunch speech_interpreter start.launch     (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - rosrun challenge_emergency emergency.py



#############################################################
## Locations that must be defined in database on forehand: ##
#############################################################
# - initial
# - meeting_point
# - other_side_room
# - exit

#################
### Questions ###
#################
# -


########################
##### STATEMACHINE #####
########################

class turn_Around_z_axis(smach.State):
    def __init__(self, robot, rotation):
        """
        @param robot
        @param rotation, the rotation in radian with which amigo needs to turn
        """
        smach.State.__init__(self, outcomes=["done", "abort"])
        self.robot    = robot
        self.rotation = rotation

    def execute(self, userdata=None):
        """ 
        @return
        """
        ##rospy.logger('Emergency.py:turnAmigo.Execute: {0} degrees.'.format((self.rotation)))

        rospy.loginfo("Executing: {0} radians".format(self.rotation))
        # get current position and rotation
        pos, rot = self.robot.base.get_location()

        # set rotation
        rot.z = rot.z - self.rotation

        # create new path
        path = self.robot.base.get_plan(pos,rot)

        # execute new path
        if self.robot.base.execute_plan(path):
            return "done"
        else:
            return "abort"

class Ask_yes_no(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["yes", "preempted", "no"])

        self.robot = robot
        self.preempted = False
        self.rate = rate
        self.get_yes_no_service = rospy.ServiceProxy('interpreter/get_yes_no', GetYesNo)

    def execute(self, userdata=None):

        self.response = self.get_yes_no_service(3 , 10) # 3 tries, each max 10 seconds

        if self.response.answer == "true":
            return "yes"
        else:
            return "no"

# Class for moving arm back for guidance
class MoveArmBack(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):      
        rospy.loginfo("Moving arm back")
    
        self.robot.leftArm.send_joint_goal(0.0,-1.57,0.0,1.57,0.0,0.0,0.0)
        self.robot.rightArm.send_joint_goal(0.0,-1.57,0.0,1.57,0.0,0.0,0.0)
        rospy.sleep(1.5)
        self.robot.leftArm.send_gripper_goal_open(10)
        self.robot.rightArm.send_gripper_goal_open(10)
        
        self.robot.speech.speak("Please grab my hand")
        return 'finished'

 # Class for registering person
class Register(smach.State):
    def __init__(self, robot=None, status=1):
        smach.State.__init__(self, outcomes=['finished'])
        # TODO ERIK register person and assert him as visited and registered.
        self.person_no = 1
        self.robot = robot

        self.status = status

    def execute(self, userdata=None):      
        rospy.loginfo("Register person in file ....")

        f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','a')  # 'a' means append
        #f = open('status.txt','a')
        if self.status == 0:
            f.write('person_%d;0;' % self.person_no)  # ;1 will say that person is not okay. ;0 is oke and ;2 vuur
            pos, rot = self.robot.base.get_location()
            f.write('%.2f;%.2f \n' % (pos.x, pos.y))
            f.close()

        elif self.status == 1:
            f.write('person_%d;1;' % self.person_no)  # ;1 will say that person is not okay. ;0 is oke and ;2 vuur
            pos, rot = self.robot.base.get_location()
            f.write('%.2f;%.2f \n' % (pos.x, pos.y))
            f.close()

        # TODO ERIK: MAKE PICTURE, SAVE AND SEND TO PDF FILE.

        self.person_no = self.person_no + 1
        return 'finished'

# Class to move arm to initial pose
class MoveArmInitial(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):   
        self.robot.speech.speak("Can you please let go of my arm")   
        rospy.loginfo("Moving arm back")
    
        self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        rospy.sleep(1.5)
        self.robot.leftArm.send_gripper_goal_close(10)
        self.robot.rightArm.send_gripper_goal_close(10)
        
        self.robot.speech.speak("We arrived at the exit, please stay here and wait for help")
        return 'finished'

def setup_statemachine(robot):

    # Retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_unreachable", "X")))

    # Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    # Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "emergency")))

    #Assert not_visited locations
    robot.reasoner.query(Compound("init_not_roi","Location"))

    # create state machine for searching smoke

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={   'initialized' : 'AT_FRONT_OF_DOOR',  ##'AT_FRONT_OF_DOOR','DETECT_PEOPLE'
                                                    'abort'       : 'Aborted'})
    
        smach.StateMachine.add('AT_FRONT_OF_DOOR',
                                    states.Say(robot, 'I will now check if the door is open or not'),
                                    transitions={   'spoken':'STATE_DOOR'}) 

        # Start laser sensor that may change the state of the door if the door is open:
        smach.StateMachine.add('STATE_DOOR', 
                                    states.Read_laser(robot, "entrance_door"),
                                    transitions={'laser_read':'WAIT_FOR_DOOR'})       
        
        # define query for the question wether the door is open in the state WAIT_FOR_DOOR
        dooropen_query = Compound("state", "entrance_door", "open")
        
        # Query if the door is open:
        smach.StateMachine.add('WAIT_FOR_DOOR', 
                                    states.Ask_query_true(robot, dooropen_query),
                                    transitions={   'query_false':'STATE_DOOR',
                                                    'query_true':'THROUGH_DOOR',
                                                    'waiting':'DOOR_CLOSED',
                                                    'preempted':'INIT_POSE'})

        # If the door is still closed after certain number of iterations, defined in Ask_query_true 
        # in perception.py, amigo will speak and check again if the door is open
        smach.StateMachine.add('DOOR_CLOSED',
                                    states.Say(robot, 'Door is closed, please open the door'),
                                    transitions={'spoken':'STATE_DOOR'}) 

        ######################################################
        ############ ENTER ROOM AND GO TO KITCHEN ############
        ######################################################
        # If the door is open, amigo will say that it goes to the registration table
        smach.StateMachine.add('THROUGH_DOOR',
                                    states.Say(robot, 'Door is open, so I will go to the kitchen'),
                                    transitions={'spoken':'INIT_POSE'}) 

        # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial pose is set.
        smach.StateMachine.add('INIT_POSE',
                                    states.Set_initial_pose(robot, 'initial'),
                                    transitions={   'done':'GO_TO_KITCHEN',
                                                    'preempted':'WAIT_FOR_DOOR',
                                                    'error':'WAIT_FOR_DOOR'})

        query_kitchen_1 = Compound("waypoint", "kitchen_1", Compound("pose_2d", "X", "Y", "Phi"))
        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_KITCHEN', 
                                    states.Navigate_to_queryoutcome(robot, query_kitchen_1, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'SAY_IS_THERE_SOMETHING_BURNING', 
                                                    'preempted':'CLEAR_PATH_TO_KITCHEN', 
                                                    'unreachable':'CLEAR_PATH_TO_KITCHEN', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_KITCHEN'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_KITCHEN',
                                    states.Say(robot, "At my first attempt I could not go to the kitchen. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_KITCHEN_SECOND_TRY'}) 

        query_kitchen_2 = Compound("waypoint", "kitchen_2", Compound("pose_2d", "X", "Y", "Phi"))
        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_KITCHEN_SECOND_TRY', 
                                    states.Navigate_to_queryoutcome(robot, query_kitchen_2, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'SAY_IS_THERE_SOMETHING_BURNING', 
                                                    'preempted':'FAIL_BUT_START_SEARCH', 
                                                    'unreachable':'FAIL_BUT_START_SEARCH', 
                                                    'goal_not_defined':'FAIL_BUT_START_SEARCH'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('FAIL_BUT_START_SEARCH',
                                    states.Say(robot, "I was still not able to go to the kitchen, but I sense that there is something burning. I will try to look for people."),
                                    transitions={'spoken':'FIND_PEOPLE'}) 

        # Say that something is burning
        smach.StateMachine.add('SAY_IS_THERE_SOMETHING_BURNING',
                                    states.Say(robot, 'Is something burning?'),
                                    transitions={'spoken':'TURN_ROUND_Z_AXIS'}) 

        ######################################################
        ################# DETECT FIRE/SMOKE ##################
        ######################################################
        '''
        TODOS: In the rulebook it is said that smoke should be detected within 1 minute and that the robot should look and point 
        from a close distance (less than 1 m) and take a picture. 
        -> Build timer 1 minute!!
        -> Detect fire
        -> Make picture
        '''

        # Turn 360 degrees to search for the smoke
        smach.StateMachine.add('TURN_ROUND_Z_AXIS',
                                    turn_Around_z_axis(robot, 10),
                                    transitions={   'done':'SAY_NEXT_LOCATION',    ###### 
                                                    'abort':'SAY_NEXT_LOCATION'})
        # Inform people
        smach.StateMachine.add('SAY_NEXT_LOCATION',
                                    states.Say(robot, 'Did not find fire. Maybe at the other side of the room'),
                                    transitions={'spoken':'NEXT_LOCATION'})

        # Query reasoner to find other side of goal
        query_kitchen_3 = Compound("waypoint", "kitchen_3", Compound("pose_2d", "X", "Y", "Phi"))
        smach.StateMachine.add('NEXT_LOCATION',
                                states.Navigate_to_queryoutcome(robot, query_kitchen_3, X="X", Y="Y", Phi="Phi"),
                                transitions={   "arrived":"TURN_AROUND_Z_AXIS2",
                                                "unreachable":'SAY_DETECT_SMOKE',
                                                "preempted":'SAY_DETECT_SMOKE',
                                                "goal_not_defined":'SAY_DETECT_SMOKE'})

        # Turn around your z-axis to detect person/smoke
        smach.StateMachine.add('TURN_AROUND_Z_AXIS2',
                                    turn_Around_z_axis(robot, 10),
                                    transitions={   'done':'WAIT_SOME_SECONDS', 
                                                    'abort':'Aborted'})
        # Wait for some seconds 
        smach.StateMachine.add('WAIT_SOME_SECONDS',
                                    states.Wait_time(robot,1),                    #adjust to one minute                  
                                    transitions={   'waited':'SAY_DETECT_SMOKE',
                                                    'preempted':'Aborted'})
        # Time ran out say, you couldn't find smoke
        smach.StateMachine.add('SAY_DETECT_SMOKE',
                                    states.Say(robot, 'Time ran out, could not find fire. But I smell something burning. Going to look for people!'),
                                    transitions={'spoken':'FIND_PEOPLE'})

        '''
        Idea: FLASH RED light on and off after detecting smoke until the end of the challenge (not during interpretation). 
        Then say: "If anyone can see or hear me, 
        try to get to the exit!!"
        '''

        ######################################################
        ################### SAVING PERSONS ###################
        ######################################################

        ############ DRIVE TO PREDEFINED LOCATIONS ###########

        smach.StateMachine.add('SAY_FIND_PEOPLE',
                                    states.Say(robot, 'I am going to look for more people.'),
                                    transitions={'spoken':'FIND_PEOPLE'})

        query_point = Compound("region_of_interest_emergency","ROI_Location", Compound("point_3d", "X", "Y", "Z"))
        object_identifier_query = "ROI_Location"

        '''
        TODO ERIK: use pose_2d to drive to a location in stead of roi. navigate_to_queryoutcome could be used, but then location visited should be asserted
        '''

        smach.StateMachine.add("FIND_PEOPLE",
                                states.Visit_query_outcome_3d(robot, 
                                                                  query_point, 
                                                                  x_offset=0.7, y_offset=0.0001,
                                                                  identifier=object_identifier_query),
                                transitions={   'arrived':'START_PEOPLE_DETECTION',
                                                'unreachable':'FAILED_DRIVING_TO_LOCATION',
                                                'preempted':'FAILED_DRIVING_TO_LOCATION',
                                                'goal_not_defined':'FAILED_DRIVING_TO_LOCATION',
                                                'all_matches_tried':'SAY_GO_TO_EXIT'})

        # Could not reach ROI     TODO: Test if unreachable location will not be driven to again.
        smach.StateMachine.add("FAILED_DRIVING_TO_LOCATION",
                                states.Say(robot,"I was not able to go to a desired location to detect people. I will try another location."),
                                transitions={'spoken':'FIND_PEOPLE'})

        ############ DRIVE TO PREDEFINED LOCATIONS ###########
        
        '''
        TODO BUILD IN PERCEPTION!!!

        - state: say I will now start looking for people
        - state: start perception (node of Jos using top laser scanner)
        - state: query people in world model
        - IF people_found
            - state: say found people, drive to it now
            - state: drive to location and go to state DETECT PEOPLE
        - ELSE 
            - state: go to state FIND_PEOPLE for next location.
        '''

        # Start people detection
        # TODO ACTUALLY USE PEOPLE DETECTION
        smach.StateMachine.add("START_PEOPLE_DETECTION",
                                states.Say(robot,"Now I should start my perception, but is not build in yet..."),
                                transitions={'spoken':'DETECT_PEOPLE'})


        ############### GET INFO STATUS PERSON ###############

        # People detection
        smach.StateMachine.add('DETECT_PEOPLE',
                                    states.Say(robot, [ "Hello, are you okay?",
                                                        "Hi, are you able to move?"]),
                                    transitions={'spoken':'ANSWER_ARE_YOU_OKAY'})
        # Await anser of question 1
        smach.StateMachine.add('ANSWER_ARE_YOU_OKAY',
                                    Ask_yes_no(robot),
                                    transitions={   'yes':'ASK_IF_KNOWS_DIRECTION',
                                                    'preempted':'SAY_REGISTER_NOT_OKAY',
                                                    'no':'SAY_REGISTER_NOT_OKAY'})      

        # Person is ok and is asked if he/she knows the way
        smach.StateMachine.add('ASK_IF_KNOWS_DIRECTION',
                                    states.Say(robot, 'Do you know the way to the exit'),
                                    transitions={'spoken':'ANSWER_IF_KNOWS_DIRECTION'})

        # Await question of he/she knows the way
        smach.StateMachine.add('ANSWER_IF_KNOWS_DIRECTION',
                                    Ask_yes_no(robot),
                                    transitions={   'yes':'SAY_REGISTER_OKAY_EXIT_BY_THEMSELVES',       ''' TODO Erik: make state in which persons that are okay are registered and give input 0 (person is not oke)" '''
                                                    'preempted':'SAY_REGISTER_OKAY_EXIT_NOT_BY_THEMSELVES',
                                                    'no':'SAY_REGISTER_OKAY_EXIT_NOT_BY_THEMSELVES'})

        ################## REGISTER PERSON ###################

        # Person is not okay:
        smach.StateMachine.add('SAY_REGISTER_NOT_OKAY',
                                    states.Say(robot, 'Okay, I will register your position and take a picture so the fire department is able to find you.'),
                                    transitions={'spoken':'REGISTER_PERSON_NOT_OKAY'})     

        smach.StateMachine.add('REGISTER_PERSON_NOT_OKAY',
                                    Register(robot, 1),                             #input 1 (person is not oke)
                                    transitions={'finished':'SAY_REGISTER_NOT_OKAY_CALM'})

        smach.StateMachine.add('SAY_REGISTER_NOT_OKAY_CALM',
                                    states.Say(robot, 'Please stay calm and help will arrive very soon.'),
                                    transitions={'spoken':'SAY_FIND_PEOPLE'})     

        # Person is ok and needs no assistance to exit
        smach.StateMachine.add('SAY_REGISTER_OKAY_EXIT_BY_THEMSELVES',
                                    states.Say(robot, 'I will register your position and take a picture.'),
                                    transitions={'spoken':'REGISTER_PERSON_OKAY_EXIT_BY_THEMSELVES'})     

        smach.StateMachine.add('REGISTER_PERSON_OKAY_EXIT_BY_THEMSELVES',
                                    Register(robot, 0),                            #input 0 (person is oke)
                                    transitions={'finished':'SAY_MOVE_TO_EXIT'})

        smach.StateMachine.add('SAY_MOVE_TO_EXIT',
                                    states.Say(robot, 'Okay, please go to the exit and leave the room'),
                                    transitions={'spoken':'SAY_FIND_PEOPLE'})

        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_REGISTER_OKAY_EXIT_NOT_BY_THEMSELVES',
                                    states.Say(robot, 'Before escorting you to the exit, I will first register your position and take a picture.'),
                                    transitions={'spoken':'REGISTER_PERSON_OKAY_EXIT_NOT_BY_THEMSELVES'})     

        smach.StateMachine.add('REGISTER_PERSON_OKAY_EXIT_NOT_BY_THEMSELVES',
                                    Register(robot, 0),                            #input 0 (person is oke)
                                    transitions={'finished':'MOVE_ARM_BACK_SAY'})
        
        ################### GUIDE TO EXIT ####################

        smach.StateMachine.add('MOVE_ARM_BACK_SAY',
                                    states.Say(robot, 'I will turn around and move my arms to the back, so that you can hold on to them'),
                                    transitions={'spoken':'MOVE_ARM_BACK_TURN'})

        # Turn 360 degrees (will be 3/4 of a round)
        smach.StateMachine.add('MOVE_ARM_BACK_TURN',
                                    turn_Around_z_axis(robot, 10),
                                    transitions={   'done':'MOVE_ARM_BACK',    ###### 
                                                    'abort':'MOVE_ARM_BACK'})
        

        # Move arm back when person needs guidance
        smach.StateMachine.add('MOVE_ARM_BACK',
                                    MoveArmBack(robot),
                                    transitions={'finished':'SAY_FOLLOW_ME'})

        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_FOLLOW_ME',
                                    states.Say(robot, 'Please hold on to one of my arms and I will drive slowely to the exit.'),
                                    transitions={'spoken':'GO_TO_FRONT_OF_EXIT'})


        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_FRONT_OF_EXIT', 
                                    states.Navigate_named(robot, "front_of_exit"),   # TODO ERIK: create location 1 meter before exit and indicate that the exit is in front of amigo.
                                    transitions={   'arrived':'SAY_GO_THROUGH_EXIT', 
                                                    'preempted':'CLEAR_PATH_TO_FRONT_OF_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_FRONT_OF_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_FRONT_OF_EXIT'})


        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_FRONT_OF_EXIT',
                                    states.Say(robot, "I could not go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_FRONT_OF_EXIT_SECOND_TRY'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_FRONT_OF_EXIT_SECOND_TRY', 
                                    states.Navigate_named(robot, "front_of_exit"),   # TODO ERIK: create location 1 meter before exit and indicate that the exit is in front of amigo.
                                    transitions={   'arrived':'SAY_GO_THROUGH_EXIT', 
                                                    'preempted':'FAILED_GO_TO_FRONT_OF_EXIT', 
                                                    'unreachable':'FAILED_GO_TO_FRONT_OF_EXIT', 
                                                    'goal_not_defined':'FAILED_GO_TO_FRONT_OF_EXIT'})

        # Person is ok and needs assistance to exit
        smach.StateMachine.add('FAILED_GO_TO_FRONT_OF_EXIT',
                                    states.Say(robot, 'I was not able to go to the exit, try to find it yourself or wait here for help since your location is registered.'),
                                    transitions={'spoken':'MOVE_ARM_INITIAL'})
        
        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_GO_THROUGH_EXIT',
                                    states.Say(robot, 'You can find the exit 1 meter in front of me, please go through it!'),
                                    transitions={'spoken':'MOVE_ARM_INITIAL'})

        # When arrived at exit move arm to natural look
        smach.StateMachine.add('MOVE_ARM_INITIAL',
                                    MoveArmInitial(robot),
                                    transitions={'finished':'SAY_FIND_PEOPLE'})

        ######################################################
        ###################### FINISHED ######################
        ######################################################
        
        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('SAY_GO_TO_EXIT',
                                    states.Say(robot, "I have searched the whole apartment for people. Therefore I will go to the exit. If people are still here and can hear me, go to the exit!!"),
                                    transitions={'spoken':'GO_TO_EXIT'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.Navigate_named(robot, "exit"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'CLEAR_PATH_TO_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_EXIT'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    states.Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_SECOND_TRY'}) 

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY', 
                                    states.Navigate_named(robot, "registration_table"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'FAILED_GO_TO_EXIT', 
                                                    'unreachable':'FAILED_GO_TO_EXIT', 
                                                    'goal_not_defined':'FAILED_GO_TO_EXIT'})

        smach.StateMachine.add('FAILED_GO_TO_EXIT',
                                    states.Say(robot, 'I was not able to go to the exit. I will stop here and save all the information I gathered in a PDF file on a USB stick.'),
                                    transitions={'spoken':'AT_END'})

        smach.StateMachine.add('SUCCEED_GO_TO_EXIT',
                                    states.Say(robot, 'I will now save all the information I gathered in a PDF file on a USB stick.'),
                                    transitions={'spoken':'AT_END'})

        '''
        TODO make state to launch start.launch -> via launch file omdat parameters.yaml ingeladen moeten worden.
        '''

        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={'spoken':'Done'})



    return sm

if __name__ == "__main__":
    rospy.init_node('emergency_exec', log_level=rospy.DEBUG)
    startup(setup_statemachine)
