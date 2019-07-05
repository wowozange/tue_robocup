# Gist of the robot-operator dialogue for te cleanup challenge

#   When the robot has picked up a object
#   Call the operator for help.
#   Say "Operator, I need some help. Would you be so kind to come to me?"
#   Wait for the operator to be detected. On timeout, repeat the call for help.
#   If operator is present
#       Say "I want to tidy up this room, but i do not know where to place the object in my gripper.
#       Say "Will you help me with that?"
#       Say "Please say YES or NO to confirm"
#
#       If YES
#         Say "Please tell me the category of this object"
#         Say "You may choose from: Candies, Cleaning Stuff, Drinks, Food, Fruits, or Snacks"
#         Say "Say Trash if the object is in none of the categories"
#         Wait for the answer of the operator (failsafe with timeout)
#         Repeat the answer and thank the operator
#         Navigate to the implied location
#         Place the object on the designated spot
#       If NO
#         Say" If you do not know the answer this must be trash, so i will take it to the trashbin"
#         Thank the operator for his assistance
#         Navigate to the trashbin location
#         Place the object in the bin
#
#   Go to the next location in the room to be examined


# THIS NEEDS TO BE ADAPTED !!!!!!!!!!!
class AskWhichRoomToClean(smach.StateMachine):
    def __init__(self, robot, room_grammar, roomw, cleanup_locationsw):
        smach.StateMachine.__init__(self, outcomes=["done"])

        hmi_result_des = ds.VariableDesignator(resolve_type=hmi.HMIResult, name="hmi_result_des")
        room_name_des = ds.FuncDesignator(ds.AttrDesignator(hmi_result_des, "semantics", resolve_type=unicode),
                                          str, resolve_type=str)

        @smach.cb_interface(outcomes=['done'])
        def write_room(ud, des_read, des_write):
            # type: (object, ds.Designator, ds.Designator) -> str
            assert(ds.is_writeable(des_write))
            assert(des_write.resolve_type == des_read.resolve_type)
            des_write.write(des_read.resolve())
            return 'done'

        with self:
            smach.StateMachine.add("ASK_WHICH_ROOM", robot_smach_states.Say(robot, "Which room should I clean for you?",
                                                                            block=True),
                                   transitions={"spoken": "HEAR_ROOM"})
            smach.StateMachine.add("HEAR_ROOM", robot_smach_states.HearOptionsExtra(robot, room_grammar,
                                                                                    ds.writeable(hmi_result_des)),
                                   transitions={"heard": "SAY_HEARD_CORRECT",
                                                "no_result": "ASK_WHICH_ROOM"})
            smach.StateMachine.add("SAY_HEARD_CORRECT", robot_smach_states.SayFormatted(
                robot, "I understood that the {room} should be cleaned, is this correct?", room=room_name_des,
                block=True),
                                   transitions={"spoken": "HEAR_CORRECT"})
            smach.StateMachine.add("HEAR_CORRECT", robot_smach_states.AskYesNo(robot),
                                   transitions={"yes": "FILL_LOCATIONS",
                                                "no": "ASK_WHICH_ROOM",
                                                "no_result": "ASK_WHICH_ROOM"})
            smach.StateMachine.add("FILL_LOCATIONS", RoomToCleanUpLocations(challenge_knowledge, room_name_des,
                                                                            cleanup_locationsw),
                                   transitions={"done": "WRITE_ROOM"})
            smach.StateMachine.add('WRITE_ROOM', smach.CBState(write_room, cb_args=[room_name_des, roomw]),
                                   transitions={'done': 'done'})

