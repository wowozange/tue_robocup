#!/usr/bin/env python

import smach, rospy, sys
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import VariableDesignator
import robot_smach_states as states

import threading
import time
import itertools

import math
from visualization_msgs.msg import Marker

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations, msg_constructors


class FollowOperator(smach.State):
    def __init__(self, robot, ask_follow=True, operator_radius=1, lookat_radius=1.5, timeout=1.0, start_timeout=10, operator_timeout=20,
                 distance_threshold=None, lost_timeout=5, lost_distance=1.5,
                 operator_id_des=VariableDesignator(resolve_type=str), standing_still_timeout=20, operator_standing_still_timeout=3.0):
        smach.State.__init__(self, outcomes=["stopped",'lost_operator', "no_operator"])
        self._robot = robot
        self._time_started = None
        self._operator = None
        self._operator_id = None
        self._operator_name = "operator"
        self._operator_radius = operator_radius
        self._lookat_radius = lookat_radius
        self._start_timeout = start_timeout
        self._breadcrumbs = []
        self._breadcrumb_distance = 0.1  # meters between dropped breadcrumbs
        self._operator_timeout = operator_timeout
        self._ask_follow = ask_follow
        self._lost_timeout = lost_timeout
        self._lost_distance = lost_distance
        self._standing_still_timeout = standing_still_timeout
        self._operator_standing_still_timeout = operator_standing_still_timeout

        self._operator_id_des = operator_id_des
        self._operator_distance = None

        self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name, geometry_msgs.msg.PointStamped, queue_size=10)
        self._plan_marker_pub = rospy.Publisher('/%s/global_planner/visualization/markers/global_plan' % robot.robot_name, Marker, queue_size=10)
        self._breadcrumb_pub = rospy.Publisher('/%s/follow_operator/breadcrumbs' % robot.robot_name, Marker, queue_size=10)

        self._last_pose_stamped = None
        self._last_operator_pose_stamped = None

        self._period = 0.5

    def _operator_standing_still_for_x_seconds(self, timeout):
        if not self._operator:
            return False

        operator_current_pose = self._operator.pose
        operator_current_pose_stamped = msg_constructors.PoseStamped(x=operator_current_pose.position.x, y=operator_current_pose.position.y)
        print "Operator position: %s" % self._operator.pose.position

        if not self._last_operator_pose_stamped:
            self._last_operator_pose_stamped = operator_current_pose_stamped
        else:
            # Compare the pose with the last pose and update if difference is larger than x
            if math.hypot(operator_current_pose_stamped.pose.position.x - self._last_operator_pose_stamped.pose.position.x, operator_current_pose_stamped.pose.position.y - self._last_operator_pose_stamped.pose.position.y) > 0.15:
                # Update the last pose
                print "Last pose stamped operator (%f,%f) at %f secs"%(self._last_operator_pose_stamped.pose.position.x, self._last_operator_pose_stamped.pose.position.y, self._last_operator_pose_stamped.header.stamp.secs)
                self._last_operator_pose_stamped = operator_current_pose_stamped
            else:
                print "Operator is standing still for %f seconds" % (operator_current_pose_stamped.header.stamp - self._last_operator_pose_stamped.header.stamp).to_sec()
                # Check whether we passed the timeout
                if (operator_current_pose_stamped.header.stamp - self._last_operator_pose_stamped.header.stamp).to_sec() > timeout:
                    return True
        return False

    def _standing_still_for_x_seconds(self, timeout):
        current_pose_stamped = self._robot.base.get_location()

        if not self._last_pose_stamped:
            self._last_pose_stamped = current_pose_stamped
        else:
            current_yaw = transformations.euler_z_from_quaternion(current_pose_stamped.pose.orientation)
            last_yaw = transformations.euler_z_from_quaternion(self._last_pose_stamped.pose.orientation)

            # Compare the pose with the last pose and update if difference is larger than x
            if math.hypot(current_pose_stamped.pose.position.x - self._last_pose_stamped.pose.position.x,
                          current_pose_stamped.pose.position.y - self._last_pose_stamped.pose.position.y) > 0.05 or abs(current_yaw - last_yaw) > 0.3:
                # Update the last pose
                print "Last pose stamped (%f,%f) at %f secs"%(self._last_pose_stamped.pose.position.x, self._last_pose_stamped.pose.position.y, self._last_pose_stamped.header.stamp.secs)
                self._last_pose_stamped = current_pose_stamped
            else:
                print "Robot is standing still :/"

                print "Robot dit not move for x seconds: %f"%(current_pose_stamped.header.stamp - self._last_pose_stamped.header.stamp).to_sec()

                # Check whether we passed the timeout
                if (current_pose_stamped.header.stamp - self._last_pose_stamped.header.stamp).to_sec() > timeout:
                    return True
        return False

    def _register_operator(self):
        start_time = rospy.Time.now()

        self._robot.head.look_at_standing_person()

        if self._operator_id:
            operator = self._robot.ed.get_entity( id=self._operator_id )
        else:
            operator = None

        while not operator:
            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return False

            if self._ask_follow:
                self._robot.speech.speak("Should I follow you?", block=True)
                answer = self._robot.ears.recognize("(yes|no)", {})

                if answer:
                    if answer.result == "yes":
                        operator = self._robot.ed.get_closest_laser_entity(radius=0.5, center_point=msg_constructors.PointStamped(x=1.0, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))
                        self._robot.speech.speak("Please look at me while I learn to recognize you. Just in case.", block=False)

                        self._robot.ed.learn_person(self._operator_name)
                        if not operator:
                            self._robot.speech.speak("Please stand in front of me")
                    elif answer.result == "no":
                        return False
                    else:
                        rospy.sleep(2)
                else:
                    self._robot.speech.speak("Something is wrong with my ears, please take a look!")
                    return False
            else:
                operator = self._robot.ed.get_closest_laser_entity(radius=1, center_point=msg_constructors.PointStamped(x=1.5, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))
                if not operator:
                    rospy.sleep(1)

        # Operator is None?
        print "We have a new operator: %s"%operator.id
        self._robot.speech.speak("Ok, I will follow you!", block=False)
        self._operator_id = operator.id
        self._breadcrumbs.append(operator)

        self._robot.head.close()

        return True

    def _update_breadcrumb_path(self):
        ''' If the last breadcrumb is less than a threshold away, replace
        the last breadcrumb with the latest operator position; otherwise
        just add it. '''
        if self._operator_id:
            if self._breadcrumbs:
                dx = self._breadcrumbs[-1].pose.position.x - self._operator.pose.position.x
                dy = self._breadcrumbs[-1].pose.position.y - self._operator.pose.position.y
                if math.hypot(dx, dy) < self._breadcrumb_distance :
                    self._breadcrumbs[-1] = self._operator
                else:
                    self._breadcrumbs.append(self._operator)
            else:
                self._breadcrumbs.append(self._operator)

        ''' Remove 'reached' breadcrumbs from breadcrumb path'''
        robot_position = self._robot.base.get_location().pose.position
        temp_crumbs = []
        for crumb in self._breadcrumbs:
            dx = crumb.pose.position.x - robot_position.x
            dy = crumb.pose.position.y - robot_position.y
            if math.hypot(dx, dy) > self._lookat_radius + 0.1:
                temp_crumbs.append(crumb)

        self._breadcrumbs = temp_crumbs

        self._visualize_breadcrumbs()

    def _track_operator(self):
        if self._operator_id:
            self._operator = self._robot.ed.get_entity( id=self._operator_id )
        else:
            self._operator = None

        if self._operator:
            # If the operator is still tracked, it is also the last_operator
            self._last_operator = self._operator

            operator_pos = geometry_msgs.msg.PointStamped()
            operator_pos.header.stamp = rospy.get_rostime()
            operator_pos.header.frame_id = self._operator_id
            operator_pos.point.x = 0.0
            operator_pos.point.y = 0.0
            operator_pos.point.z = 0.0
            self._operator_pub.publish(operator_pos)

            robot_position = self._robot.base.get_location().pose.position
            operator_position = self._last_operator.pose.position

            dx = operator_position.x - robot_position.x
            dy = operator_position.y - robot_position.y
            self._operator_distance = math.hypot(dx, dy)

            return True
        else:
            # If the operator is lost, check if we still have an ID
            if self._operator_id:
                # At the moment when the operator is lost, tell him to slow down and clear operator ID
                self._operator_id = None
                self._robot.speech.speak("Stop! I lost you!", block=False)
            return False

    def _visualize_breadcrumbs(self):
        breadcrumbs_msg = Marker()
        breadcrumbs_msg.type = Marker.POINTS
        breadcrumbs_msg.scale.x = 0.05
        breadcrumbs_msg.header.stamp = rospy.get_rostime()
        breadcrumbs_msg.header.frame_id = "/map"
        breadcrumbs_msg.color.a = 1
        breadcrumbs_msg.color.r = 0
        breadcrumbs_msg.color.g = 1
        breadcrumbs_msg.color.b = 1
        breadcrumbs_msg.lifetime = rospy.Time(1.0)
        breadcrumbs_msg.id = 0
        breadcrumbs_msg.action = Marker.ADD

        for crumb in self._breadcrumbs:
            breadcrumbs_msg.points.append(crumb.pose.position)

        self._breadcrumb_pub.publish(breadcrumbs_msg)

    def _visualize_plan(self, path):
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.scale.x = 0.05
        line_strip.header.frame_id = "/map"
        line_strip.header.stamp = rospy.Time.now()
        line_strip.color.a = 1
        line_strip.color.r = 0
        line_strip.color.g = 1
        line_strip.color.b = 1
        line_strip.id = 0
        line_strip.action = Marker.ADD

        # Push back all pnts
        for pose_stamped in path:
            line_strip.points.append(pose_stamped.pose.position)

        self._plan_marker_pub.publish(line_strip)

    def _update_navigation(self):
        self._robot.head.cancel_goal()

        robot_position = self._robot.base.get_location().pose.position
        operator_position = self._last_operator.pose.position

        ''' Define end goal constraint, solely based on the (old) operator position '''
        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2"% (operator_position.x, operator_position.y, self._operator_radius)

        o = OrientationConstraint()
        if self._operator_id:
            o.frame = self._operator_id
        else:
            o.frame = 'map'
            o.look_at = self._last_operator.pose.position

        ''' Calculate global plan from robot position, through breadcrumbs, to the operator '''
        res = 0.05
        plan = []
        previous_point = robot_position

        breadcrumbs = self._breadcrumbs + [self._operator]
        for crumb in breadcrumbs:
            dx = crumb.pose.position.x - previous_point.x
            dy = crumb.pose.position.y - previous_point.y

            length = math.hypot(dx, dy)

            if length != 0:
                dx_norm = dx / length
                dy_norm = dy / length
                yaw = math.atan2(dy, dx)

                start = 0
                end = int(length / res)

                for i in range(start, end):
                    x = previous_point.x + i * dx_norm * res
                    y = previous_point.y + i * dy_norm * res
                    plan.append(msg_constructors.PoseStamped(x=x, y=y, z=0, yaw=yaw))

            previous_point = crumb.pose.position

        # Delete the elements from the plan within the operator radius from the robot
        cutoff = int(self._operator_radius/(2.0*res))
        if len(plan) > cutoff:
            del plan[-cutoff:]

        ''' Check if plan is valid. If not, remove invalid points from the path '''
        if not self._robot.base.global_planner.checkPlan(plan):
            print "Breadcrumb plan is blocked"
            # Go through plan from operator to robot and pick the first unoccupied point as goal point
            plan = [point for point in plan if self._robot.base.global_planner.checkPlan([point])]

        self._visualize_plan(plan)
        self._robot.base.local_planner.setPlan(plan, p, o)

    def _recover_operator(self):
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("%s, please stand in front of me and look at me" % self._operator_name, block=False)

        # Find faces
        detections = self._robot.ed.detect_persons()
        if not detections:
            return False

        # recognize operator and then find closest entity to face.
        for detection in detections:
            if detection.name is self._operator_name:
                recovered_operator = self._robot.ed.get_closest_laser_entity(radius=self._lost_distance,
                                                                             center_point=detection.pose.pose.position)

        if recovered_operator:
            self._operator_id = recovered_operator.id
            self._operator = recovered_operator
            return True

        return False

    def _turn_towards_operator(self):
        robot_position = self._robot.base.get_location().pose.position
        operator_position = self._last_operator.pose.position

        ''' Define end goal constraint, solely based on the (old) operator position '''
        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2"% (operator_position.x, operator_position.y, self._operator_radius)

        o = OrientationConstraint()
        if self._operator_id:
            o.frame = self._operator_id
        else:
            o.frame = 'map'
            o.look_at = self._last_operator.pose.position

        ''' Determine if the goal has been reached. If it has, return True '''
        dx = operator_position.x - robot_position.x
        dy = operator_position.y - robot_position.y

        yaw = math.atan2(dy, dx)
        plan = [msg_constructors.PoseStamped(x=robot_position.x, y=robot_position.y, z=0, yaw=yaw)]
        print "Operator within self._lookat_radius"

        self._robot.base.local_planner.setPlan(plan, p, o)

    def _check_end_criteria(self):
        # Check if we still have an operator
        lost_operator = self._operator is None

        # Try to recover operator if lost and reached last seen operator position
        if lost_operator and self._operator_distance < self._operator_radius:
            if not self._recover_operator():
                self._robot.base.local_planner.cancelCurrentPlan()
                return "lost_operator"

        # Check are standing still long
        if self._standing_still_for_x_seconds(self._standing_still_timeout):
            self._robot.base.local_planner.cancelCurrentPlan()
            return "lost_operator"

        # Check if we are already there (in operator radius and operator standing still long enough)
        if self._operator_distance < self._operator_radius and self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout):
            if (self._robot.base.get_location().header.stamp - self._time_started).to_sec() > self._start_timeout:
                # Check if we pass the start timeout
                self._operator_id_des.writeable.write(self._operator_id)
                self._robot.base.local_planner.cancelCurrentPlan()
                return "stopped"

        # No end criteria met
        return None

    def execute(self, userdata):
        # Reset robot and operator last pose
        self._last_pose_stamped = None
        self._last_operator_pose_stamped = None
        self._breadcrumbs = []

        if self._operator_id_des:
            operator_id = self._operator_id_des.resolve()
            if operator_id:
                self._operator_id = operator_id

        self._robot.head.close()

        if self._robot.robot_name == "amigo":
            self._robot.torso.send_goal('reset', timeout=4.0)

        if not self._register_operator():
            self._robot.base.local_planner.cancelCurrentPlan()
            return "no_operator"

        self._time_started = rospy.Time.now()

        while not rospy.is_shutdown():

            # 1) Track operator
            self._track_operator()

            # 2) Keep track of operator history
            self._update_breadcrumb_path()

            # 3) Check end criteria
            result = self._check_end_criteria()
            if result:
                return result

            # 4) Action
            if not self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout) and self._operator_distance < self._lookat_radius:
                self._turn_towards_operator()
            else:
                self._update_navigation()

            rospy.sleep(self._period) # Loop at 2Hz

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        smach.StateMachine.add('TEST', FollowOperator(robot), transitions={"stopped":"TEST",'lost_operator':"TEST", "no_operator":"TEST"})
        return sm

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)
