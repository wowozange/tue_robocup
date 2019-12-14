import rospy
import tf2_ros
from hsrb_interface import settings, joint_group
from hsrb_interface import robot, robot_model, trajectory, utils
from hsrb_interface._extension import KinematicsInterface
from hsrb_interface.joint_group import JointGroup
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

_DEBUG = False

# Timeout for motion planning [sec]
_PLANNING_ARM_TIMEOUT = 10.0

# Timeout to receive a tf message [sec]
_TF_TIMEOUT = 5.0


def _init_jointgroup(self, name):
    super(JointGroup, self).__init__()
    self._setting = settings.get_entry('joint_group', name)
    self._position_control_clients = []
    arm_config = self._setting['arm_controller_prefix']
    print("Creating arm controller")
    self._position_control_clients.append(
        trajectory.TrajectoryController(arm_config))
    head_config = self._setting['head_controller_prefix']
    print("Creating head controller")
    self._position_control_clients.append(
        trajectory.TrajectoryController(head_config))
    hand_config = self._setting["hand_controller_prefix"]
    print("Creating hand controller")
    self._position_control_clients.append(
        trajectory.TrajectoryController(hand_config))
    base_config = self._setting["omni_base_controller_prefix"]
    print("Creating base controller")
    self._base_client = trajectory.TrajectoryController(
        base_config, "/base_coordinates")
    self._position_control_clients.append(self._base_client)
    # Disabled impedance controller
    # imp_config = settings.get_entry("trajectory", "impedance_control")
    # print("Creating impedance controller")
    # self._impedance_client = trajectory.ImpedanceController(imp_config)
    joint_state_topic = self._setting["joint_states_topic"]
    print("Joint state topic: {}".format(joint_state_topic))
    self._joint_state_sub = utils.CachingSubscriber(
        joint_state_topic,
        JointState,
        default=JointState())
    timeout = self._setting.get('timeout', None)
    self._joint_state_sub.wait_for_message(timeout)
    self._tf2_buffer = robot._get_tf2_buffer()
    self._end_effector_frames = self._setting['end_effector_frames']
    self._end_effector_frame = self._end_effector_frames[0]
    self._passive_joints = self._setting['passive_joints']
    print("Creating URDF")
    self._robot_urdf = robot_model.RobotModel.from_parameter_server(
        key='/hero/robot_description')  # Added namespace
    print("Getting robot description")
    description = rospy.get_param('/hero/robot_description')  # Added namespace
    self._kinematics_interface = KinematicsInterface(description)

    self._collision_world = None
    self._linear_weight = 3.0
    self._angular_weight = 1.0
    self._joint_weights = {}
    self._planning_timeout = _PLANNING_ARM_TIMEOUT
    self._use_base_timeopt = True
    self._looking_hand_constraint = False
    self._tf_timeout = _TF_TIMEOUT

    if _DEBUG:
        self._vis_pub = rospy.Publisher("tsr_marker", MarkerArray,
                                        queue_size=1)
        self._tf2_pub = tf2_ros.TransformBroadcaster()
    print("Interface done")


def update_joint_group():
    joint_group.JointGroup.__init__ = _init_jointgroup


# class ConfigurableJointGroup(JointGroup):
#     def __init__(self, name, tf_listener):
#         """
#         Inherits from hsrb_interface JointGroup in order to make it configurable
#
#         This opposes the hardcoded 'settings' in the hsrb_interface package
#
#         :param name: name of the joint group (e.g., "whole_body")
#         """
#         super(JointGroup, self).__init__()
#         self._setting = hsrb_settings.get_entry('joint_group', name)
#         self._position_control_clients = []
#         arm_config = self._setting['arm_controller_prefix']
#         print("Creating arm controller")
#         self._position_control_clients.append(
#             trajectory.TrajectoryController(arm_config))
#         head_config = self._setting['head_controller_prefix']
#         print("Creating head controller")
#         self._position_control_clients.append(
#             trajectory.TrajectoryController(head_config))
#         hand_config = self._setting["hand_controller_prefix"]
#         print("Creating hand controller")
#         self._position_control_clients.append(
#             trajectory.TrajectoryController(hand_config))
#         base_config = self._setting["omni_base_controller_prefix"]
#         print("Creating base controller")
#         self._base_client = trajectory.TrajectoryController(
#             base_config, "/base_coordinates")
#         self._position_control_clients.append(self._base_client)
#         imp_config = settings.get_entry("trajectory", "impedance_control")
#         print("Creating impedance controller")
#         self._impedance_client = trajectory.ImpedanceController(imp_config)
#         joint_state_topic = self._setting["joint_states_topic"]
#         print("Joint state topic: {}".format(joint_state_topic))
#         self._joint_state_sub = utils.CachingSubscriber(
#             joint_state_topic,
#             JointState,
#             default=JointState())
#         timeout = self._setting.get('timeout', None)
#         self._joint_state_sub.wait_for_message(timeout)
#         self._tf2_buffer = robot._get_tf2_buffer()
#         self._end_effector_frames = self._setting['end_effector_frames']
#         self._end_effector_frame = self._end_effector_frames[0]
#         self._passive_joints = self._setting['passive_joints']
#         print("Creating URDF")
#         self._robot_urdf = robot_model.RobotModel.from_parameter_server(
#             key='/robot_description')
#         print("Getting robot description")
#         description = rospy.get_param('/robot_description')
#         self._kinematics_interface = KinematicsInterface(description)
#
#         self._collision_world = None
#         self._linear_weight = 3.0
#         self._angular_weight = 1.0
#         self._joint_weights = {}
#         self._planning_timeout = _PLANNING_ARM_TIMEOUT
#         self._use_base_timeopt = True
#         self._looking_hand_constraint = False
#         self._tf_timeout = _TF_TIMEOUT
#
#         if _DEBUG:
#             self._vis_pub = rospy.Publisher("tsr_marker", MarkerArray,
#                                             queue_size=1)
#             self._tf2_pub = tf2_ros.TransformBroadcaster()
#         print("Interface done")
#
