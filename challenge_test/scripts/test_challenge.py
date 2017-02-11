#!/usr/bin/python

# System
import sys

# ROS
import rospy
import smach

# TU/e
import robot_smach_states
# import ipdb; ipdb.set_trace()
from challenge_test.clean_inspect import CleanInspect
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_test')


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               robot_smach_states.Initialize(robot),
                               transitions={"initialized": "SAY_START_CHALLENGE",
                                            "abort": "Aborted"})

        smach.StateMachine.add('SAY_START_CHALLENGE',
                               robot_smach_states.Say(robot, ["Starting R5COP Cooperative cleaning demonstrator",
                                                              "What a mess here, let's clean this room!",
                                                              "Let's see if I can find some garbage here",
                                                              "All I want to do is clean this mess up!"], block=False),
                               transitions={"spoken": "INSPECT_0"})

        for i, place in enumerate(challenge_knowledge.inspection_places):
            next_i = i + 1 if i + 1 < len(challenge_knowledge.inspection_places) else 0

            smach.StateMachine.add("INSPECT_%d" % i,
                                   CleanInspect(robot, place["entity_id"], place["room_id"], place["navigate_area"],
                                                place["segment_areas"], challenge_knowledge.known_types),
                                   transitions={"done": "INSPECT_%d" % next_i})
    return sm

if __name__ == '__main__':
    rospy.init_node('r5cop_demo_amigo')

    # Check if we have something specified to inspect
    if len(challenge_knowledge.inspection_places) < 1:
        rospy.logerr("The challenge knowledge inspection_places list should contain at least one entry!")
        sys.exit(1)

    robot_smach_states.util.startup(setup_statemachine, challenge_name="r5cop_demo_amigo")
