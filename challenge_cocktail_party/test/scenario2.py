#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

from fast_simulator.client import *

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = SimWorld()

    raw_input("\nPress enter to say 'John'\n")
    W.speak("john")

    raw_input("\nPress enter to say 'yes'\n")
    W.speak("yes")

    raw_input("\nPress enter to say 'milk'\n")
    W.speak("milk")

    raw_input("\nPress enter to say 'yes'\n")
    W.speak("yes")

