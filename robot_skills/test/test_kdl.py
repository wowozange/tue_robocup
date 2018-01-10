#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy
from copy import deepcopy

import robot_skills.util.kdl_conversions as kdl_conversions
import PyKDL as kdl

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print("Please specify a robot name")
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("kdl_test")

robot = robot_constructor(robot_name)

print("\nrobot.base.get_location().frame")
print(robot.base.get_location().frame)

print("\nrobot.base.get_location().frame.p")
print(robot.base.get_location().frame.p)

print("\nlocation = robot.base.get_location()")
print("location.frame.p")
location = robot.base.get_location()
print(location.frame.p)

print("\nframe = robot.base.get_location().frame")
print("frame.p")
frame = robot.base.get_location().frame
print(frame.p)

print("\np = robot.base.get_location().frame.p")
print("p")
p = robot.base.get_location().frame.p
print(p)

print("\nrobot.base.get_location().extractVectorStamped()")
print(robot.base.get_location().extractVectorStamped())

print("\nVS = robot.base.get_location().extractVectorStamped()")
print("VS")
VS = robot.base.get_location().extractVectorStamped()
print(VS)

print("\nlocation = robot.base.get_location()")
print("location.extractVectorStamped()")
print(location.extractVectorStamped())

print("\nVS = location.extractVectorStamped()")
print("VS")
VS = location.extractVectorStamped()
print(VS)

print('\nfs = kdl_conversions.FrameStamped(kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3)), "/map")')
print("fs.extractVectorStamped()")
fs = kdl_conversions.FrameStamped(kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3)), "/map")
print(fs.extractVectorStamped())

import PyKDL as kdl
print(kdl.__file__)
rot = kdl.Rotation.Quaternion(1, 0, 0, 0)
trans = kdl.Vector(1, 2, 3)
frame=kdl.Frame(rot, trans)
print("\n\n")
print("\nframe.p")
print(frame.p)
print("\nkdl.Frame(frame)")
print(kdl.Frame(frame))
print("\nkdl.Frame(frame).p")
print(kdl.Frame(frame).p)
