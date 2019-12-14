# ROS
import rospy
import tf

# Toyota
# ToDo: add dependencies
from hsrb_interface import geometry
from hsrb_interface import Robot
from tmc_manipulation_msgs.msg import ArmManipulationErrorCodes, BaseMovementType
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest

# Robot skills
from robot_skills.util.kdl_conversions import FrameStamped
from ..core import RobotPart
from .joint_group import update_joint_group

from .hsrb_robot import update_hsrb_settings


class HeroArm(RobotPart):
    def __init__(self, robot_name, tf_listener, get_joint_states):
        # type: (str, tf.TransformListener, callable) -> None
        """
        Arm interface for Hero (HSR) robot

        :param robot_name: robot name
        :param tf_listener: tf_listener
        :param get_joint_states: method for getting the last joint states
        """
        super(HeroArm, self).__init__(robot_name, tf_listener)

        # The following is *copied* from Arm and should therefore move to a better location
        self.default_configurations = self.load_param('skills/arm/default_configurations')
        self.default_trajectories   = self.load_param('skills/arm/default_trajectories')

        # Fix namespacing
        # hsrb_settings._HSRB_SETTINGS = hsrb_settings._HSRB_SETTINGS.replace("/hsrb/", "/hero/")
        # print(hsrb_settings._HSRB_SETTINGS)
        # from .hsrb_robot import Robot
        print("Constructing robot")
        update_hsrb_settings()
        update_joint_group()
        self._hsrb_robot_interface = Robot()  # ToDo: construct robot object in Hero?
        print("Constructing interface")
        self._whole_body_interface = self._hsrb_robot_interface.try_get("whole_body")
        print("Hero Arm init done")
        # self._whole_body_interface = ConfigurableJointGroup("whole_body", tf_listener)

    def send_goal(self, frameStamped, timeout=30, pre_grasp=False, first_joint_pos_only=False,
                  allowed_touch_objects=None):
        # type: (FrameStamped, float, bool, bool, list) -> bool
        """
        Send a arm to a goal:

        Using a combination of position and orientation: a kdl.Frame. A time
        out time_out. pre_grasp means go to an offset that is normally needed
        for things such as grasping. You can also specify the frame_id which
        defaults to base_link

        :param frameStamped: A FrameStamped to move the arm's end effector to
        :param timeout: timeout in seconds; In case of 0.0, goal is executed without feedback and waiting
        :param pre_grasp: Bool to use pre_grasp or not
        :param first_joint_pos_only: Bool to only execute first joint position of whole trajectory
        :param allowed_touch_objects: List of object names in the worldmodel, which are allowed to be touched
        :return: True of False
        """
        # The following code has been copied from the manipulation bridge and modified.
        # Further improvements can be done in a later stage

        # This line is completely different from the original
        pose = [frameStamped.frame.p.x(), frameStamped.frame.p.y(), frameStamped.frame.p.z()],\
            list(frameStamped.frame.M.GetQuaternion())

        # ref_frame_id = hsrb_settings.get_frame("base")  # Original
        ref_frame_id = frameStamped.frame_id

        ref_to_hand_poses = [pose]

        odom_to_ref_pose = self._whole_body._lookup_odom_to_ref(ref_frame_id)
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_hand_poses = []
        for ref_to_hand in ref_to_hand_poses:
            odom_to_hand = geometry.multiply_tuples(odom_to_ref, ref_to_hand)
            odom_to_hand_poses.append(geometry.tuples_to_pose(odom_to_hand))

        rospy.loginfo("Generating planning request: {}".format(PlanWithHandGoalsRequest))
        req = self._whole_body._generate_planning_request(PlanWithHandGoalsRequest)
        req.origin_to_hand_goals = odom_to_hand_poses
        req.ref_frame_id = self.whole_body._end_effector_frame
        req.base_movement_type.val = BaseMovementType.ROTATION_Z

        service_name = self.whole_body._setting['plan_with_hand_goals_service']
        plan_service = rospy.ServiceProxy(service_name, PlanWithHandGoals)
        rospy.loginfo("Sending planning request to moveit: {}".format(req))
        res = plan_service.call(req)
        rospy.loginfo("Planning result: {}".format(res))
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            rospy.logerr('Fail to plan move_endpoint')
            success = False
        else:
            res.base_solution.header.frame_id = hsrb_settings.get_frame('odom')
            constrained_traj = self._whole_body._constrain_trajectories(res.solution, res.base_solution)
            self._whole_body._execute_trajectory(constrained_traj)

        return success

