#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import rospy
from robot_skills import Hero
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState


class PickItemFromCupboardDrawer(StateMachine):
    def __init__(self, robot, cupboard_id, required_items):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'], output_keys=["item_picked"])
        arm = robot.get_arm()._arm
        picked_items = []

        def send_joint_goal(position_array, wait_for_motion_done=True):
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string):
            arm.send_gripper_goal(open_close_string)
            rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=['succeeded', 'failed'], output_keys=["item_picked"])
        def _ask_user(user_data):
            leftover_items = [item for item in required_items if item not in picked_items]
            if not leftover_items:
                robot.speech.speak("We picked 'm all apparently")
                return 'failed'

            arm.send_joint_goal("carrying_pose")

            item_name = leftover_items[0]
            picked_items.append(item_name)

            robot.speech.speak("Please put the {} in my gripper".format(item_name), block=False)
            send_gripper_goal("open")
            rospy.sleep(5.0)
            robot.speech.speak("Thanks for that!", block=False)
            send_gripper_goal("close")

            # Set output data
            user_data['item_picked'] = item_name

            arm.send_joint_goal("carrying_pose")

            return 'succeeded'

        with self:
            self.add('ASK_USER', CBState(_ask_user), transitions={'succeeded': 'succeeded', 'failed': 'failed'})


class NavigateToAndPickItemFromCupboardDrawer(StateMachine):
    def __init__(self, robot, cupboard_id, cupboard_navigation_area, required_items):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], output_keys=["item_picked"])

        cupboard = EdEntityDesignator(robot=robot, id=cupboard_id)

        with self:
            StateMachine.add("NAVIGATE_TO_CUPBOARD",
                             NavigateToSymbolic(robot, {cupboard: cupboard_navigation_area}, cupboard),
                             transitions={'arrived': 'PICK_ITEM_FROM_CUPBOARD',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("PICK_ITEM_FROM_CUPBOARD", PickItemFromCupboardDrawer(robot, cupboard_id, required_items),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    NavigateToAndPickItemFromCupboardDrawer(hero, 'cupboard', 'aside_of').execute()