# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robot_skills.classification_result import ClassificationResult
from robocup_knowledge import knowledge_loader

# Challenge serving drinks
from .serve_one_drink import ServeOneDrink
from .sd_states import AskAvailability

# Knowledge
challenge_knowledge = knowledge_loader.load_knowledge("challenge_serving_drinks")
common_knowledge = knowledge_loader.load_knowledge("common")


class CheckInspect(smach.State):
    def __init__(self, designator, *resolve_types):
        super(CheckInspect, self).__init__(outcomes=["true", "false"])
        ds.check_type(designator, *resolve_types)
        self.designator = designator

    def execute(self, userdata=None):
        val = self.designator.resolve() if hasattr(self.designator, "resolve") else self.designator
        if val:
            return "true"
        else:
            return "false"


class UpdateUnavailableDrinkList(smach.State):
    def __init__(self, unavailable_drink_list_designator, drink_to_add_designator):
        super(UpdateUnavailableDrink, self).__init__(outcomes=["done", "failed"])
        ds.is_writable(unavailable_drink_list_designator)
        ds.check_type(unavailable_drink_list_designator, [str])
        ds.check_type(drink_to_add_designator, str)

        self.unavailable_drink_list_designator = unavailable_drink_list_designator
        self.drink_to_add_designator = drink_to_add_designator

    def execute(self, userdata=None):
        drink_to_add = self.drink_to_add_designator.resolve() if hasattr(self.drink_to_add_designator, "resolve") else self.drink_to_add_designator

        if not drink_to_add:
            return "failed"
        else:
            unavailable_drink_list = self.unavailable_drink_list_designator.resolve() if hasattr(self.unavailable_drink_list_designator, "resolve") else self.unavailable_drink_list_designator
            if drink_to_add not in unavailable_drink_list:
                unavailable_drink_list.append(drink_to_add)
                self.unavailable_drink_list_designator.write(unavailable_drink_list)

            return "done"


class IdentifyUnavailableDrinkFromRecognitions(smach.State):
    def __init__(self, objects, objects_list_des, unavailable_drink_designator):
        super(IdentifyUnavailableDrinkFromRecognitions, self).__init__(outcomes=["done", "failed"])
        ds.is_writable(unavailable_drink_designator)
        ds.check_type(unavailable_drink_designator, str)
        ds.check_type(objects_list_des, [ClassificationResult])

        self._objects = objects
        self._objects_list_des = objects_list_des
        self._unavailable_drink_designator = unavailable_drink_designator

    def execute(self, userdata=None):
        objects_classification_list = self._objects_list_des.resolve() if hasattr(self._objects_list_des, "resolve") else self._objects_list_des

        for common_object in self._objects:


class ServingDrinks(smach.StateMachine):
    """
    State machine for 'Serving Drinks' challenge.
    """

    def __init__(self, robot):
        # type: (Robot) -> str
        """
        Initialization method
        :param robot: robot api object
        """

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # Designators
        bar_designator = ds.EdEntityDesignator(robot=robot, id=challenge_knowledge.bar_id, name='bar_des')
        room_designator = ds.EdEntityDesignator(robot=robot, id=challenge_knowledge.room_id, name='room_des')

        objects_list_des = ds.VariableDesignator(resolve_type=[ClassificationResult], name='objects_list_des')
        unav_drink_des = ds.VariableDesignator(resolve_type=str, name='unav_drink_str_des')

        with self:
            smach.StateMachine.add("INITIALIZE",
                                   states.Initialize(robot=robot),
                                   transitions={"initialized": "INITIAL_POSE",
                                                "abort": "aborted"})

            smach.StateMachine.add("INITIAL_POSE",
                                   states.SetInitialPose(robot,
                                                         challenge_knowledge.starting_point),
                                   transitions={"done": "INSPECT_BAR",
                                                "preempted": "aborted",
                                                "error": "INSPECT_BAR"})

            # Inspect bar and store the list of available drinks
            smach.StateMachine.add("INSPECT_BAR",
                                   states.Inspect(robot=robot,
                                                  entityDes=bar_designator,
                                                  navigation_area="in_front_of",
                                                  objectIDsDes=objects_list_des),
                                   transitions={"done": "CHECK_INSPECT_RESULT",
                                                "failed": "INSPECT_FALLBACK"})

            smach.StateMachine.add("CHECK_INSPECT_RESULT",
                                   CheckInspect(objects_list_des,
                                                [ClassificationResult]),
                                   transitions={"true": "NAVIGATE_TO_ROOM",
                                                "false": "INSPECT_FALLBACK"})

            # Inspect fallback - ask the bartender which drink is unavailable and store the unavailable drink
            smach.StateMachine.add("INSPECT_FALLBACK",
                                   AskAvailability(robot=robot,
                                                   unavailable_drink_designator=unav_drink_des.writeable,
                                                   objects=common_knowledge.objects),
                                   transitions={"succeeded": "NAVIGATE_TO_ROOM",
                                                "failed": "NAVIGATE_TO_ROOM"})

            # Navigate to the predefined room
            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   states.NavigateToRoom(robot=robot, entity_designator_room=room_designator),
                                   transitions={"arrived": "SAY_HI",
                                                "unreachable": "SAY_HI",
                                                "goal_not_defined": "aborted"})

            smach.StateMachine.add("SAY_HI",
                                   states.Say(robot, "Hi, I am {}. I'll be your waiter today".format(robot.robot_name)),
                                   transitions={"spoken": "SERVE_DRINK_1"})

            # Explicitly add a new state for each drink, i.e., don't use a range iterator to make sure a new state
            # is constructed every time
            for idx in range(1, challenge_knowledge.NR_DRINKS + 1):
                next_state = "SERVE_DRINK_{}".format(idx + 1) if idx < challenge_knowledge.NR_DRINKS else "SAY_DONE"

                smach.StateMachine.add("SERVE_DRINK_{}".format(idx),
                                       ServeOneDrink(robot=robot,
                                                     bar_designator=bar_designator,
                                                     room_id=challenge_knowledge.room_id,
                                                     room_designator=room_designator,
                                                     objects_list_des=objects_list_des,
                                                     unav_drink_des=unav_drink_des,
                                                     name_options=common_knowledge.names,
                                                     objects=common_knowledge.objects),
                                       transitions={"succeeded": next_state,
                                                    "failed": next_state,
                                                    "aborted": "aborted"})

            smach.StateMachine.add("SAY_DONE",
                                   states.Say(robot, "My job here is done. Enjoy your day and see you next time"),
                                   transitions={"spoken": "succeeded"})
