#!/usr/bin/env python
import rospy
from robot_skills.util.robot_constructor import robot_constructor

rospy.init_node('test_mode_down_until_force_sensor_edge_up', anonymous=True)
robot = robot_constructor("hero")
arm = robot.get_arm(available_force_sensor=True)

arm.move_down_until_force_sensor_edge_up()
