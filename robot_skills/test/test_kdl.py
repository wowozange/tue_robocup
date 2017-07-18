#! /usr/bin/env python

import sys
import rospy

import robot_skills.util.kdl_conversions as kdl_conversions
import PyKDL as kdl

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("kdl_test")

robot = robot_constructor(robot_name)

print "\nrobot.base.get_location().frame.p"
print robot.base.get_location().frame.p

print "\nlocation = robot.base.get_location()"
print "location.frame.p"
location = robot.base.get_location()
print location.frame.p

print "\nframe = robot.base.get_location().frame"
print "frame.p"
frame = robot.base.get_location().frame
print frame.p

print "\np = robot.base.get_location().frame.p"
print "p"
p = robot.base.get_location().frame.p
print p

print "\nrobot.base.get_location().extractVectorStamped()"
print robot.base.get_location().extractVectorStamped()

print "\nVS = robot.base.get_location().extractVectorStamped()"
print "VS"
VS = robot.base.get_location().extractVectorStamped()
print VS

print "\nlocation = robot.base.get_location()"
print "location.extractVectorStamped()"
print location.extractVectorStamped()

print "\nVS = location.extractVectorStamped()"
print "VS"
VS = location.extractVectorStamped()
print VS

print '\nfs = kdl_conversions.FrameStamped(kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3)), "/map")'
print "fs.extractVectorStamped()"
fs = kdl_conversions.FrameStamped(kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3)), "/map")
print fs.extractVectorStamped()
