#! /usr/bin/env python

# System
import argparse
import csv
import time

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped
from robot_smach_states import Grab, Inspect, ClassificationResult, NavigateToWaypoint, ForceDrive, SetGripper, \
    Say

try:
    from typing import List
except ImportError:
    pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Measure the time it takes to grasp an entity when starting from a given pose. "
                                                 "After the grasping is done, go back to the waypoint, "
                                                 "turn around and drop the item, "
                                                 "then turn around once more to be in the start configuration again")
    parser.add_argument("cls", type=str,
                        help="class of entity to grasp, eg. 'coke' or '*' if you don't care")
    parser.add_argument("support", type=str,
                        help="ID of entity to grasp FROM ('grasp' entity is on-top-of this 'support' entity), eg. 'cabinet'")
    parser.add_argument("waypoint", type=str,
                        help="ID of waypoint for start and end position of the robot")
    parser.add_argument("--inspect_from_area", type=str,
                        help="What area of the support-entity to inspect from", default='in_front_of')
    parser.add_argument("--robot", default="hero",
                        help="Robot name (amigo, hero, sergio)")
    parser.add_argument("--output", default="grasp_benchmark.csv",
                        help="Output of the benchmark results")
    parser.add_argument("--non-strict-class", action='store_true',
                        help="Only grasp an item if it matches the specified class. "
                             "Pass --non-strict-class=False to also grab another class of items "
                             "if none of the expected class is observed. "
                             "Use this if you only care about grasping, not object recognition")

    args = parser.parse_args()
    rospy.init_node("benchmark_grasping")

    with open(args.output, 'a+') as csv_file:
        reader = csv.DictReader(csv_file)

        fields = ['robot',
                  'start_waypoint',
                  'expected_class', 'observed_class',
                  'id',
                  'inspect_duration',
                  'grab_duration',
                  'x', 'y', 'z']
        results_writer = csv.DictWriter(csv_file, fieldnames=fields, extrasaction='ignore')  # Ignore extra keys in the dicts

        if reader.fieldnames != results_writer.fieldnames:
            results_writer.writeheader()

        robot = get_robot(args.robot)

        grasp_cls = ds.Designator(args.cls, name='grasp_cls')
        support_entity = ds.EdEntityDesignator(robot, id=args.support, name='support_entity')
        entity_ids = ds.VariableDesignator([], resolve_type=[ClassificationResult], name='entity_ids')
        waypoint = ds.EdEntityDesignator(robot, id=args.waypoint, name='waypoint')

        arm = ds.LockingDesignator(ds.UnoccupiedArmDesignator(robot, {}))
        arm.lock()

        record = {'robot': args.robot, 'start_waypoint': args.waypoint, 'expected_class': args.cls,
                  'id': None}

        try:
            nav_to_start = NavigateToWaypoint(robot,
                                              waypoint_designator=waypoint,
                                              look_at_designator=support_entity)

            assert nav_to_start.execute() == 'arrived', "I did not arrive at the start"

            inspect = Inspect(robot=robot, entityDes=support_entity, objectIDsDes=entity_ids,
                              navigation_area=args.inspect_from_area)

            record['inspect_start'] = '{:.3f}'.format(time.time())
            assert inspect.execute() == 'done', "I could not inspect the support entity"
            record['inspect_end'] = '{:.3f}'.format(time.time())
            record['inspect_duration'] = '{:3.3f}'.format(float(record['inspect_end']) - float(record['inspect_start']))

            inspection_result = entity_ids.resolve()  # type: List[ClassificationResult]
            if inspection_result:
                if grasp_cls.resolve() == '*' or args.non_strict_class:
                    rospy.loginfo("Any item will match")
                    matching_results = inspection_result
                else:
                    matching_results = [result for result in inspection_result if result.type == grasp_cls.resolve()]
                    rospy.loginfo("Found {} items of class {}".format(len(matching_results), grasp_cls.resolve()))


                if matching_results:
                    if len(matching_results) > 1:
                        rospy.logwarn("There are multiple items OK to grab, will select the last one")
                        record['observed_class'] = ' '.join([result.type for result in matching_results])
                    else:
                        record['observed_class'] = matching_results[-1].type
                    selected_entity_id = matching_results[-1].id

                    rospy.loginfo("Selected entity {} for grasping".format(selected_entity_id))
                    grasp_entity = ds.EdEntityDesignator(robot, id=selected_entity_id)
                    record['id'] = selected_entity_id[:6]

                    entity = grasp_entity.resolve() # type: Entity
                    if entity:
                        vector_stamped = entity.pose.extractVectorStamped()  # type: VectorStamped
                        record['x'] = '{:.3f}'.format(vector_stamped.vector.x())
                        record['y'] = '{:.3f}'.format(vector_stamped.vector.y())
                        record['z'] = '{:.3f}'.format(vector_stamped.vector.z())

                    grab_state = Grab(robot, grasp_entity, arm)

                    record['grab_start'] = '{:.3f}'.format(time.time())
                    assert grab_state.execute() == 'done', "I couldn't grasp"
                    record['grab_end'] = '{:.3f}'.format(time.time())
                    record['grab_duration'] = '{:3.3f}'.format(float(record['grab_end']) - float(record['grab_start']))

                    assert nav_to_start.execute() == 'arrived', "I could not navigate back to the start"

                    rospy.logwarn("Robot will turn around to drop the {}".format(grasp_cls.resolve()))

                    force_drive = ForceDrive(robot, 0, 0, 1, 3.14)  #rotate 180 degs in pi seconds
                    force_drive.execute()

                    say_drop = Say(robot, sentence="I'm going to drop the item, please hold it!")
                    say_drop.execute()

                    drop_it = SetGripper(robot, arm_designator=arm, grab_entity_designator=grasp_entity)
                    drop_it.execute()

                    force_drive_back = ForceDrive(robot, 0, 0, -1, 3.14)  #rotate -180 degs in pi seconds
                    force_drive_back.execute()
                    nav_to_start.execute()
                else:
                    rospy.logerr("No entities found of the given class '{}'".format(grasp_cls.resolve()))
            else:
                rospy.logerr("No entities found at all :-(")
        except AssertionError as assertion_err:
            say_fail = Say(robot, sentence=assertion_err.message)
            say_fail.execute()
        finally:
            results_writer.writerow(record)
