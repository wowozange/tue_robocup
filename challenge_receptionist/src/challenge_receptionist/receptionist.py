import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import smach
from hmi import HMIResult
from robocup_knowledge import load_knowledge
from robot_skills.util.entity import Entity
from challenge_receptionist.find_empty_seat import FindEmptySeat
from challenge_receptionist.learn_guest import LearnGuest, FieldOfHMIResult

challenge_knowledge = load_knowledge('challenge_receptionist')


class GuestDescriptionStrDesignator(ds.Designator):
    def __init__(self, guest_name_des, drinkname, name=None):
        super(GuestDescriptionStrDesignator, self).__init__(resolve_type=str, name=name)

        ds.check_type(guest_name_des, str)
        ds.check_type(drinkname, str)

        self.guest_name_des = guest_name_des
        self.drinkname = drinkname

    def _resolve(self):
        name = self.guest_name_des.resolve()
        drinkname = self.drinkname.resolve()
        return "This is {name} whose favourite drink is {drink}".format(name=name, drink=drinkname)


class IntroduceGuestToOperator(smach.StateMachine):
    def __init__(self, robot, guest_ent_des, guest_name_des, guest_drinkname_des):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        ds.check_type(guest_name_des, str)
        ds.check_type(guest_drinkname_des, str)
        ds.check_type(guest_ent_des, Entity)

        all_old_guests = ds.VariableDesignator(resolve_type=[Entity])
        current_old_guest = ds.VariableDesignator(resolve_type=Entity)

        # TODO: iterate over all people in the room (excluding the guest, but that should be behind us and thus not visible)
        # For each person:
        #   0. Go to the person (old guest)
        #   1. Look at the person and point at the guest
        #   2. Say 'Hi <person name>, this is <guest name>

        with self:
            # OLD
            # smach.StateMachine.add('FIND_OLD_GUESTS',
            #                        states.FindPersonInRoom(robot,
            #                                                # We're looking for any person and assume that thats the operator
            #                                                discard_other_labels=False,
            #                                                area=challenge_knowledge.waypoint_livingroom['id'],
            #                                                name=challenge_knowledge.operator_name,
            #                                                found_entity_designator=operator_des.writeable),
            #                        transitions={'found': 'GOTO_OPERATOR',
            #                                     'not_found': 'GOTO_OPERATOR'})
            # NEW, TODO, use new FindPersonInRoom made by Arpit
            smach.StateMachine.add('FIND_OLD_GUESTS',
                                    states.FindPersonInRoom(robot,
                                        # We're looking for any person and assume they are old guests already there
                                        discard_other_labels=False,
                                        area=challenge_knowledge.waypoint_livingroom['id'],
                                        name=challenge_knowledge.operator_name,
                                        found_entity_designator=all_old_guests.writeable),
                                    transitions = {'found': 'GOTO_OPERATOR',
                                                   'not_found': 'GOTO_OPERATOR'})

            smach.StateMachine.add('ITERATE_OLD_GUESTS',
                                   states.NavigateToObserve(robot,
                                                            all_old_guests,
                                                            current_old_guest.writeable),
                                   transitions={'next': 'GOTO_OPERATOR',
                                                'stop_iteration': 'succeeded'})

            smach.StateMachine.add('GOTO_OPERATOR',
                                   states.NavigateToObserve(robot,
                                                            current_old_guest),
                                   transitions={'arrived': 'SAY_LOOK_AT_GUEST',
                                                'unreachable': 'SAY_LOOK_AT_GUEST',
                                                'goal_not_defined': 'abort'})

            smach.StateMachine.add('SAY_LOOK_AT_GUEST',
                                   states.SayFormatted(robot,
                                                       ["Hi {name}, let me show you our guest"],
                                                       name=ds.AttrDesignator(current_old_guest, "person_properties.name", resolve_type=str),
                                                       block=True),
                                   transitions={'spoken': 'FIND_GUEST'})

            smach.StateMachine.add('FIND_GUEST',
                                   states.FindPerson(robot=robot,
                                                     person_label=guest_name_des,
                                                     found_entity_designator=guest_ent_des.writeable),
                                   transitions={"found": "POINT_AT_GUEST",
                                                "failed": "abort"})

            smach.StateMachine.add('POINT_AT_GUEST',
                                   states.PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
                                                  point_at_designator=guest_ent_des,
                                                  look_at_designator=current_old_guest),
                                   transitions={"succeeded": "INTRODUCE_GUEST",
                                                "failed": "abort"})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   states.Say(robot, GuestDescriptionStrDesignator(guest_name_des, guest_drinkname_des),
                                              block=True,
                                              look_at_standing_person=False),
                                   transitions={'spoken': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                                   states.ResetArmsTorsoHead(robot),
                                   transitions={'done': 'ITERATE_OLD_GUESTS'})


class ChallengeReceptionist(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        self.door_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_door['id'])
        self.livingroom_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_livingroom['id'])

        self.operator_designator = ds.VariableDesignator(resolve_type=Entity)

        self.guest1_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest1_entity')
        self.guest1_name_des = ds.VariableDesignator('guest 1', name='guest1_name')
        self.guest1_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest1_drink')
        self.guest1_drinkname_des = FieldOfHMIResult(self.guest1_drink_des, semantics_field='drink',
                                                     name='guest1_drinkname')

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SET_INITIAL_POSE',
                                                'abort': 'aborted'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   states.SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'LEARN_GUEST_1',
                                                "preempted": 'aborted',
                                                'error': 'LEARN_GUEST_1'})

            smach.StateMachine.add('LEARN_GUEST_1',
                                   LearnGuest(robot, self.door_waypoint, self.guest1_entity_des, self.guest1_name_des,
                                              self.guest1_drink_des),
                                   transitions={'succeeded': 'SAY_GOTO_OPERATOR',
                                                'aborted': 'aborted',
                                                'failed': 'aborted'})

            smach.StateMachine.add('SAY_GOTO_OPERATOR',
                                   states.Say(robot, ["Okidoki, lets go to {}. Please follow me".format(challenge_knowledge.operator_name)],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'GOTO_LIVINGROOM_1'})

            smach.StateMachine.add('GOTO_LIVINGROOM_1',
                                   states.NavigateToWaypoint(robot,
                                                             self.livingroom_waypoint,
                                                             challenge_knowledge.waypoint_livingroom['radius']),
                                   transitions={'arrived': 'INTRODUCE_GUEST',
                                                'unreachable': 'INTRODUCE_GUEST',
                                                'goal_not_defined': 'aborted'})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   IntroduceGuestToOperator(robot, self.operator_designator, self.guest1_entity_des,
                                                            self.guest1_name_des, self.guest1_drinkname_des),
                                   transitions={'succeeded': 'FIND_SEAT_FOR_GUEST',
                                                'abort': 'FIND_SEAT_FOR_GUEST'})

            smach.StateMachine.add('FIND_SEAT_FOR_GUEST',
                                   FindEmptySeat(robot,
                                                 seats_to_inspect=challenge_knowledge.seats,
                                                 room=ds.EntityByIdDesignator(robot, challenge_knowledge.sitting_room)),
                                   transitions={'succeeded': 'SAY_DONE',
                                                'failed': 'SAY_DONE'})

            smach.StateMachine.add('SAY_DONE',
                                   states.Say(robot, ["That's all folks, my job is done, bye bye!"],
                                              block=False),
                                   transitions={'spoken': 'succeeded'})

            # - [x] Wait at the door, say you're waiting
            # - [x] Wait until person can come in
            # - [x] Ask their name
            # - [x] Ask their favourite drink
            # - [x] Ask for favourite drink <drink1>
            # - [x] GOTO living room
            # - [x] Locate John (not sure how that should work, maybe just FindPersonInRoom)
            # - [x] GOTO John
            # - [x] Locate guest1:
            # - [x]   rotate head until <guest1> is detected
            # - [x] Point at guest1
            # - [x] Say: This is <guest1> and (s)he likes to drink <drink1>
            # - [ ] Iterate to guest 2
            # - [x] Point at empty chair for guest to sit in
